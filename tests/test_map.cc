// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests of the map that owns keyframes and map points and frees those erased
// once no thread refers to them.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <format>
#include <memory>
#include <new>
#include <optional>
#include <print>
#include <set>
#include <string_view>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "Check.h"
#include "Frames.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapPoint.h"
#include "Scene.h"

namespace {

// Addresses whose deallocation the tests observe through the global
// deallocation functions that this program replaces.
constexpr std::size_t watchedAddresses = 8;
std::array<std::atomic<const void*>, watchedAddresses> watched{};
std::array<std::atomic<bool>, watchedAddresses> released{};

void recordRelease(const void* address) noexcept
{
    for (std::size_t i = 0; i < watchedAddresses; ++i) {
        if (address != nullptr && watched[i].load() == address) {
            released[i] = true;
        }
    }
}

} // namespace

void* operator new(std::size_t size)
{
    if (void* const address = std::malloc(size == 0 ? 1 : size)) {
        return address;
    }

    throw std::bad_alloc{};
}

void operator delete(void* address) noexcept
{
    recordRelease(address);
    std::free(address);
}

void operator delete(void* address, std::size_t) noexcept
{
    recordRelease(address);
    std::free(address);
}

namespace {

using ORB_SLAM::tests::check;

using namespace std::chrono_literals;

// Calls to reclaim that cannot free anything until a participant passes a
// quiescent point, however often they repeat.
constexpr int reclamations = 4;
// Quiescent points of the only participant after which the map freed what
// was erased before them.
constexpr int quiescentPoints = 3;
// Generous enough that a loaded machine does not stall a worker past it.
constexpr auto deadline = 10s;
// Long enough for an idle worker to reach its wait.
constexpr auto pauseObservation = 50ms;

// Polls the predicate until it holds or the deadline passes, and returns
// whether it held.
template<class Predicate>
bool waitUntil(Predicate predicate)
{
    const auto until = std::chrono::steady_clock::now() + deadline;

    while (!predicate()) {
        if (std::chrono::steady_clock::now() >= until) {
            return false;
        }

        std::this_thread::yield();
    }

    return true;
}

//! Watches the deallocation of the object at @p address.
class Watch
{
public:
    explicit Watch(const void* address)
        : index_{freeSlot()}
    {
        released[index_] = false;
        watched[index_] = address;
    }

    ~Watch()
    {
        watched[index_] = nullptr;
    }

    Watch(const Watch&) = delete;
    Watch& operator=(const Watch&) = delete;

    //! Returns whether the object was freed.
    bool freed() const
    {
        return released[index_];
    }

private:
    static std::size_t freeSlot()
    {
        for (std::size_t i = 0; i < watchedAddresses; ++i) {
            if (watched[i].load() == nullptr) {
                return i;
            }
        }

        std::abort();
    }

    std::size_t index_;
};

void reclaim(ORB_SLAM::Map& map)
{
    for (int i = 0; i < reclamations; ++i) {
        map.Reclaim();
    }
}

ORB_SLAM::KeyFrame* addKeyFrame(ORB_SLAM::tests::Scene& scene)
{
    const std::vector<cv::KeyPoint> keyPoints(2, cv::KeyPoint{0, 0, 1});
    return scene.addKeyFrame(cv::Mat::eye(4, 4, CV_32F),
                             cv::Mat::eye(3, 3, CV_32F), keyPoints);
}

//! Adds a map point that the map owns, with @p keyFrame as its reference.
ORB_SLAM::MapPoint* addMapPoint(ORB_SLAM::tests::Scene& scene,
                                ORB_SLAM::KeyFrame* keyFrame)
{
    auto* const point = new ORB_SLAM::MapPoint{
        cv::Mat{cv::Vec3f{0, 0, 1}}, keyFrame, &scene.map()};
    scene.map().AddMapPoint(point);
    return point;
}

// An erased object stays allocated until every participant passed a
// quiescent point after its erasure and another one after the map removed
// the references to it. Then the map frees it.
template<class Erase>
void freedAfterQuiescentPoints(ORB_SLAM::tests::Scene& scene, const void* object,
                               Erase erase, std::string_view what)
{
    ORB_SLAM::Map::Participant participant{scene.map()};
    const Watch watch{object};

    erase();
    reclaim(scene.map());

    check(!watch.freed(),
          std::format("an erased {} stays allocated while a participant "
                      "passed no quiescent point",
                      what));

    participant.Quiesce();
    reclaim(scene.map());

    check(!watch.freed(),
          std::format("an erased {} stays allocated while a participant "
                      "passed a single quiescent point",
                      what));

    participant.Quiesce();
    reclaim(scene.map());

    check(watch.freed(),
          std::format("an erased {} is freed once the participant passed two "
                      "quiescent points",
                      what));
}

void erasedMapPointFreed()
{
    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const keyFrame = addKeyFrame(scene);
    ORB_SLAM::MapPoint* const point = addMapPoint(scene, keyFrame);
    keyFrame->AddMapPoint(point, 0);
    point->AddObservation(keyFrame, 0);

    freedAfterQuiescentPoints(
        scene, point, [point] { point->SetBadFlag(); }, "map point");
}

void replacedMapPointFreed()
{
    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const keyFrame = addKeyFrame(scene);
    ORB_SLAM::MapPoint* const replaced = addMapPoint(scene, keyFrame);
    ORB_SLAM::MapPoint* const replacement = addMapPoint(scene, keyFrame);
    keyFrame->AddMapPoint(replaced, 0);
    replaced->AddObservation(keyFrame, 0);

    freedAfterQuiescentPoints(
        scene, replaced,
        [replaced, replacement] { replaced->Replace(replacement); },
        "map point that another replaced");
}

void erasedKeyFrameFreed()
{
    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const root = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const keyFrame = addKeyFrame(scene);
    keyFrame->ChangeParent(root);

    freedAfterQuiescentPoints(
        scene, keyFrame, [keyFrame] { keyFrame->SetBadFlag(); }, "keyframe");
}

// Without participants, nothing refers to an erased object between calls,
// so that repeated reclamation frees it.
void erasedObjectFreedWithoutParticipants()
{
    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const keyFrame = addKeyFrame(scene);
    ORB_SLAM::MapPoint* const point = addMapPoint(scene, keyFrame);
    const Watch watch{point};

    point->SetBadFlag();
    reclaim(scene.map());

    check(watch.freed(),
          "an erased map point is freed without participants");
}

// A keyframe or map point that another thread updated while the object was
// erased may keep a reference to it that the erasure missed. The map removes
// such references before it frees the object.
void staleReferencesRemoved()
{
    constexpr int weight = 20;

    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const root = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const erased = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const neighbor = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const child = addKeyFrame(scene);
    erased->ChangeParent(root);
    neighbor->ChangeParent(root);

    // References that the erased objects do not know of.
    ORB_SLAM::MapPoint* const erasedPoint = addMapPoint(scene, neighbor);
    neighbor->AddMapPoint(erasedPoint, 0);
    neighbor->AddConnection(erased, weight);
    neighbor->AddChild(erased);
    neighbor->AddLoopEdge(erased);
    child->ChangeParent(erased);
    erased->EraseChild(child);

    ORB_SLAM::MapPoint* const observer = addMapPoint(scene, erased);
    observer->AddObservation(erased, 1);
    scene.map().SetReferenceMapPoints({erasedPoint, observer});

    erasedPoint->SetBadFlag();
    erased->SetBadFlag();
    reclaim(scene.map());

    check(neighbor->GetMapPoint(0) == nullptr,
          "a keyframe keeps no erased map point");
    check(!neighbor->GetConnectedKeyFrames().contains(erased),
          "a keyframe keeps no connection to an erased keyframe");
    check(!std::ranges::contains(neighbor->GetVectorCovisibleKeyFrames(), erased),
          "a keyframe keeps no erased covisible keyframe");
    check(!neighbor->GetChilds().contains(erased),
          "a keyframe keeps no erased child");
    check(!neighbor->GetLoopEdges().contains(erased),
          "a keyframe keeps no loop edge to an erased keyframe");
    check(child->GetParent() == root,
          "the child of an erased keyframe takes its parent");
    check(root->GetChilds().contains(child),
          "the parent of an erased keyframe adopts its children");
    check(!observer->GetObservations().contains(erased),
          "a map point keeps no observation of an erased keyframe");
    check(observer->GetReferenceKeyFrame() != erased,
          "a map point keeps no erased reference keyframe");

    const std::vector<ORB_SLAM::MapPoint*> references =
        scene.map().GetReferenceMapPoints();
    check(!std::ranges::contains(references, erasedPoint),
          "the map keeps no erased reference map point");
}

// The map takes no erased map point among its reference map points, which
// it would keep after freeing it.
void erasedReferenceMapPointsRefused()
{
    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const keyFrame = addKeyFrame(scene);
    ORB_SLAM::MapPoint* const point = addMapPoint(scene, keyFrame);

    point->SetBadFlag();
    scene.map().SetReferenceMapPoints({point});

    check(scene.map().GetReferenceMapPoints().empty(),
          "the map takes no erased reference map point");
}

// An object erased after a batch keeps no reference to the batch, so that
// it reads nothing freed while it waits for its own batch.
void laterErasedObjectsForgetFreedOnes()
{
    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const root = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const later = addKeyFrame(scene);
    later->ChangeParent(root);
    ORB_SLAM::MapPoint* const point = addMapPoint(scene, root);
    // A reference that the map point does not know of.
    later->AddMapPoint(point, 0);

    ORB_SLAM::Map::Participant participant{scene.map()};
    const Watch watch{point};

    point->SetBadFlag();
    reclaim(scene.map());
    later->SetBadFlag();
    participant.Quiesce();
    reclaim(scene.map());
    participant.Quiesce();
    reclaim(scene.map());

    check(watch.freed(), "the erased map point is freed");
    check(later->GetMapPoint(0) == nullptr,
          "a keyframe erased after the map point keeps no reference to it");
}

// A map point erased after a batch keeps no observation by the batch.
void laterErasedMapPointsForgetFreedKeyFrames()
{
    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const root = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const keyFrame = addKeyFrame(scene);
    keyFrame->ChangeParent(root);
    ORB_SLAM::MapPoint* const later = addMapPoint(scene, root);
    // An observation that the keyframe does not know of.
    later->AddObservation(keyFrame, 0);

    ORB_SLAM::Map::Participant participant{scene.map()};

    keyFrame->SetBadFlag();
    reclaim(scene.map());
    later->SetBadFlag();
    participant.Quiesce();
    reclaim(scene.map());

    check(!later->GetObservations().contains(keyFrame),
          "a map point erased after the keyframe keeps no observation by it");
}

// A keyframe erased twice is freed once.
void keyFrameErasedTwiceFreedOnce()
{
    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const root = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const keyFrame = addKeyFrame(scene);
    keyFrame->ChangeParent(root);
    const Watch watch{keyFrame};

    keyFrame->SetBadFlag();
    keyFrame->SetBadFlag();
    reclaim(scene.map());

    check(watch.freed(), "a keyframe erased twice is freed");
}

// Clearing the map frees the erased objects that wait for a grace period.
void clearedMapFreesErasedObjects()
{
    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const keyFrame = addKeyFrame(scene);
    ORB_SLAM::MapPoint* const waiting = addMapPoint(scene, keyFrame);
    ORB_SLAM::MapPoint* const erased = addMapPoint(scene, keyFrame);
    const Watch waitingWatch{waiting};
    const Watch erasedWatch{erased};

    // The participant never passes a quiescent point, so the first map point
    // waits for its grace period, and the second one for its batch to start.
    ORB_SLAM::Map::Participant participant{scene.map()};
    waiting->SetBadFlag();
    reclaim(scene.map());
    erased->SetBadFlag();

    scene.map().clear();

    check(waitingWatch.freed(),
          "clearing frees an erased map point that waits for its grace period");
    check(erasedWatch.freed(),
          "clearing frees an erased map point whose batch has not started");
}

// A child whose parent and grandparent are erased together takes the
// closest ancestor that remains.
void childTakesRemainingAncestor()
{
    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const root = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const grandparent = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const parent = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const child = addKeyFrame(scene);
    grandparent->ChangeParent(root);
    parent->ChangeParent(grandparent);
    // A child that its parent does not know of, which the erasure of the
    // parent therefore leaves in place.
    child->ChangeParent(parent);
    parent->EraseChild(child);

    parent->SetBadFlag();
    grandparent->SetBadFlag();
    reclaim(scene.map());

    check(child->GetParent() == root,
          "a child takes the closest ancestor that remains");
}

// Exposes the keyframes that the database indexes by each word.
class InspectedDatabase : public ORB_SLAM::KeyFrameDatabase
{
public:
    using ORB_SLAM::KeyFrameDatabase::KeyFrameDatabase;

    bool indexes(ORB_SLAM::KeyFrame* keyFrame) const
    {
        return std::ranges::any_of(mvInvertedFile, [keyFrame](const auto& keyFrames) {
            return std::ranges::contains(keyFrames, keyFrame);
        });
    }

    void index(ORB_SLAM::KeyFrame* keyFrame, DBoW2::WordId word)
    {
        mvInvertedFile[word].push_back(keyFrame);
    }
};

// The database indexes no erased keyframe, and the map removes one that
// another thread indexed while it was erased.
void databaseForgetsErasedKeyFrames()
{
    constexpr int branching = 2;
    constexpr int depth = 1;
    constexpr DBoW2::WordId word = 0;
    constexpr DBoW2::WordValue weight = 1;

    ORB_SLAM::ORBVocabulary vocabulary;
    vocabulary.create(
        {{cv::Mat::zeros(1, ORB_SLAM::tests::descriptorBytes, CV_8U),
          cv::Mat::ones(1, ORB_SLAM::tests::descriptorBytes, CV_8U)}},
        branching, depth);
    InspectedDatabase database{vocabulary};
    ORB_SLAM::Map map;
    ORB_SLAM::Frame frame =
        ORB_SLAM::tests::makeFrame(cv::Mat::eye(4, 4, CV_32F));

    auto* const root = new ORB_SLAM::KeyFrame{frame, &map, &database};
    map.AddKeyFrame(root);
    auto* const keyFrame = new ORB_SLAM::KeyFrame{frame, &map, &database};
    map.AddKeyFrame(keyFrame);
    keyFrame->ChangeParent(root);
    keyFrame->mBowVec[word] = weight;

    keyFrame->SetBadFlag();
    database.add(keyFrame);

    check(!database.indexes(keyFrame),
          "the database indexes no erased keyframe");

    // An entry that an addition racing with the erasure leaves behind.
    database.index(keyFrame, word);
    reclaim(map);

    check(!database.indexes(keyFrame),
          "the database keeps no keyframe that the map freed");
}

// The map frees the keyframes and map points it holds when it is destroyed.
void destroyedMapFreesObjects()
{
    ORB_SLAM::Frame frame =
        ORB_SLAM::tests::makeFrame(cv::Mat::eye(4, 4, CV_32F));
    std::optional<ORB_SLAM::Map> map{std::in_place};

    auto* const keyFrame = new ORB_SLAM::KeyFrame{frame, &*map, nullptr};
    map->AddKeyFrame(keyFrame);
    auto* const point =
        new ORB_SLAM::MapPoint{cv::Mat{cv::Vec3f{0, 0, 1}}, keyFrame, &*map};
    map->AddMapPoint(point);

    const Watch keyFrameWatch{keyFrame};
    const Watch pointWatch{point};

    map.reset();

    check(keyFrameWatch.freed(), "a destroyed map frees its keyframes");
    check(pointWatch.freed(), "a destroyed map frees its map points");
}

// Exposes the pointers that local mapping keeps from one keyframe to the
// next.
class ReferringLocalMapping : public ORB_SLAM::LocalMapping
{
public:
    using ORB_SLAM::LocalMapping::LocalMapping;
    using ORB_SLAM::LocalMapping::DropErasedObjects;
    using ORB_SLAM::LocalMapping::mlpRecentAddedMapPoints;
    using ORB_SLAM::LocalMapping::mpCurrentKeyFrame;
};

// A keyframe that waits for local mapping keeps the map points that tracking
// matched, which do not know of it. At its quiescent point, local mapping
// removes those erased since.
void queuedKeyFramesDropErasedMatches()
{
    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const root = addKeyFrame(scene);
    ORB_SLAM::MapPoint* const point = addMapPoint(scene, root);

    ORB_SLAM::Frame frame = ORB_SLAM::tests::makeFrame(
        cv::Mat::eye(4, 4, CV_32F), cv::Mat{}, {cv::KeyPoint{0, 0, 1}});
    frame.mvpMapPoints = {point};
    // No worker processes the keyframe, so the test keeps owning it.
    ORB_SLAM::KeyFrame keyFrame{frame, &scene.map(), &scene.database()};

    ReferringLocalMapping localMapping{&scene.map()};
    localMapping.InsertKeyFrame(&keyFrame);
    point->SetBadFlag();
    localMapping.DropErasedObjects();

    check(keyFrame.GetMapPoint(0) == nullptr,
          "a queued keyframe keeps no erased map point");
}

// Each pause of a running local mapping lets it pass a quiescent point,
// after which it keeps no erased object. Local mapping frees the erased
// objects once it passed enough quiescent points.
void localMappingFreesErasedObjects()
{
    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const root = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const erased = addKeyFrame(scene);
    erased->ChangeParent(root);
    ORB_SLAM::MapPoint* const point = addMapPoint(scene, root);
    const Watch watch{point};

    ReferringLocalMapping localMapping{&scene.map()};
    localMapping.mlpRecentAddedMapPoints = {point};
    localMapping.mpCurrentKeyFrame = erased;

    point->SetBadFlag();
    erased->SetBadFlag();

    {
        std::jthread mapping{[&localMapping](std::stop_token stopToken) {
            localMapping.Run(stopToken);
        }};

        for (int i = 0; i < quiescentPoints; ++i) {
            localMapping.RequestPause();
            localMapping.WaitUntilPaused(mapping.get_stop_token());
            localMapping.Resume();
        }
    }

    check(localMapping.mlpRecentAddedMapPoints.empty(),
          "local mapping keeps no erased recent map point");
    check(localMapping.mpCurrentKeyFrame == nullptr,
          "local mapping keeps no erased current keyframe");
    check(watch.freed(),
          std::format("local mapping frees an erased map point after {} "
                      "quiescent points",
                      quiescentPoints));
}

// Exposes how local mapping processes a keyframe.
class ProcessingLocalMapping : public ORB_SLAM::LocalMapping
{
public:
    using ORB_SLAM::LocalMapping::LocalMapping;
    using ORB_SLAM::LocalMapping::ProcessNewKeyFrame;
};

// A keyframe keeps the map points that tracking matched, which do not know
// of it until local mapping processes it. If one was erased meanwhile, the
// map may have removed the references to it already and frees it at the
// next quiescent point, so the processed keyframe keeps no reference to it.
void processedKeyFrameDropsErasedMatches()
{
    constexpr int branching = 2;
    constexpr int depth = 1;
    // Local mapping takes the map points of the first two keyframes from
    // the tracker as they are.
    constexpr unsigned long laterKeyFrameId = 2;

    ORB_SLAM::ORBVocabulary vocabulary;
    vocabulary.create(
        {{cv::Mat::zeros(1, ORB_SLAM::tests::descriptorBytes, CV_8U),
          cv::Mat::ones(1, ORB_SLAM::tests::descriptorBytes, CV_8U)}},
        branching, depth);
    ORB_SLAM::tests::Scene scene{vocabulary};
    ORB_SLAM::KeyFrame* const root = addKeyFrame(scene);
    ORB_SLAM::MapPoint* const point = addMapPoint(scene, root);
    point->SetBadFlag();

    ORB_SLAM::Frame frame = ORB_SLAM::tests::makeFrame(
        cv::Mat::eye(4, 4, CV_32F), cv::Mat{}, {cv::KeyPoint{0, 0, 1}});
    frame.mpORBvocabulary = &vocabulary;
    frame.mvpMapPoints = {point};
    ORB_SLAM::KeyFrame::nNextId = laterKeyFrameId;
    auto* const keyFrame =
        new ORB_SLAM::KeyFrame{frame, &scene.map(), &scene.database()};

    ProcessingLocalMapping localMapping{&scene.map()};
    localMapping.InsertKeyFrame(keyFrame);
    // Processing adds the keyframe to the map, which owns it then.
    localMapping.ProcessNewKeyFrame();

    check(keyFrame->GetMapPoint(0) == nullptr,
          "a processed keyframe keeps no erased map point");
}

// Exposes the pointers that loop closing keeps from one keyframe to the
// next.
class ReferringLoopClosing : public ORB_SLAM::LoopClosing
{
public:
    using ORB_SLAM::LoopClosing::LoopClosing;
    using ORB_SLAM::LoopClosing::CheckNewKeyFrames;
    using ORB_SLAM::LoopClosing::DropErasedObjects;
    using ORB_SLAM::LoopClosing::mpCurrentKF;
    using ORB_SLAM::LoopClosing::mpMatchedKF;
    using ORB_SLAM::LoopClosing::mvConsistentGroups;
    using ORB_SLAM::LoopClosing::mvpCurrentConnectedKFs;
    using ORB_SLAM::LoopClosing::mvpCurrentMatchedPoints;
    using ORB_SLAM::LoopClosing::mvpEnoughConsistentCandidates;
    using ORB_SLAM::LoopClosing::mvpLoopMapPoints;
};

// A loop closing that takes a keyframe passes a quiescent point after it,
// at which it keeps no erased object. Until then, the map keeps erased
// objects allocated.
void loopClosingDropsErasedObjects()
{
    // Loop closing only adds keyframes this close to the start of the map
    // to the database, which leaves its other state as it is.
    ORB_SLAM::KeyFrame::nNextId = 0;
    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const root = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const erased = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const keyFrame = addKeyFrame(scene);
    erased->ChangeParent(root);
    ORB_SLAM::MapPoint* const point = addMapPoint(scene, root);
    const Watch watch{point};

    ORB_SLAM::ORBVocabulary vocabulary;
    ReferringLoopClosing loopClosing{&scene.map(), &scene.database(),
                                     &vocabulary};
    loopClosing.mpMatchedKF = erased;
    loopClosing.mvConsistentGroups = {{{root, erased}, 1}};
    loopClosing.mvpEnoughConsistentCandidates = {root, erased};
    loopClosing.mvpCurrentConnectedKFs = {root, erased};
    loopClosing.mvpCurrentMatchedPoints = {point};
    loopClosing.mvpLoopMapPoints = {point};

    point->SetBadFlag();
    erased->SetBadFlag();
    reclaim(scene.map());

    check(!watch.freed(),
          "an erased map point stays allocated while loop closing passed no "
          "quiescent point");

    {
        std::jthread closing{[&loopClosing](std::stop_token stopToken) {
            loopClosing.Run(stopToken);
        }};

        // Each keyframe that loop closing takes lets it pass a quiescent
        // point.
        check(waitUntil([&loopClosing, keyFrame, &scene, &watch] {
                  if (!loopClosing.CheckNewKeyFrames()) {
                      loopClosing.InsertKeyFrame(keyFrame);
                  }

                  scene.map().Reclaim();
                  return watch.freed();
              }),
              "an erased map point is freed once loop closing took keyframes");
    }

    using Group = std::set<ORB_SLAM::KeyFrame*>;
    check(loopClosing.mpMatchedKF == nullptr,
          "loop closing keeps no erased matched keyframe");
    check(loopClosing.mvConsistentGroups.size() == 1 &&
              loopClosing.mvConsistentGroups[0].first == Group{root},
          "the consistent groups keep no erased keyframe");
    check(loopClosing.mvpEnoughConsistentCandidates ==
              std::vector<ORB_SLAM::KeyFrame*>{root},
          "the loop candidates keep no erased keyframe");
    check(loopClosing.mvpCurrentConnectedKFs ==
              std::vector<ORB_SLAM::KeyFrame*>{root},
          "the connected keyframes keep no erased keyframe");
    check(loopClosing.mvpCurrentMatchedPoints ==
              std::vector<ORB_SLAM::MapPoint*>{nullptr},
          "the matched map points keep no erased map point");
    check(loopClosing.mvpLoopMapPoints.empty(),
          "the loop map points keep no erased map point");
}

// Loop closing takes no erased keyframe from its queue and keeps no erased
// current keyframe after its quiescent point.
void loopClosingDropsErasedKeyFrames()
{
    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const root = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const queued = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const current = addKeyFrame(scene);
    queued->ChangeParent(root);
    current->ChangeParent(root);

    ORB_SLAM::ORBVocabulary vocabulary;
    ReferringLoopClosing loopClosing{&scene.map(), &scene.database(),
                                     &vocabulary};
    loopClosing.InsertKeyFrame(queued);
    loopClosing.mpCurrentKF = current;

    queued->SetBadFlag();
    current->SetBadFlag();
    loopClosing.DropErasedObjects();

    check(!loopClosing.CheckNewKeyFrames(),
          "loop closing keeps no erased keyframe in its queue");
    check(loopClosing.mpCurrentKF == nullptr,
          "loop closing keeps no erased current keyframe");
}

} // namespace

int main()
{
    erasedMapPointFreed();
    replacedMapPointFreed();
    erasedKeyFrameFreed();
    erasedObjectFreedWithoutParticipants();
    staleReferencesRemoved();
    erasedReferenceMapPointsRefused();
    laterErasedObjectsForgetFreedOnes();
    laterErasedMapPointsForgetFreedKeyFrames();
    keyFrameErasedTwiceFreedOnce();
    clearedMapFreesErasedObjects();
    childTakesRemainingAncestor();
    databaseForgetsErasedKeyFrames();
    destroyedMapFreesObjects();
    queuedKeyFramesDropErasedMatches();
    localMappingFreesErasedObjects();
    loopClosingDropsErasedObjects();
    loopClosingDropsErasedKeyFrames();
    processedKeyFrameDropsErasedMatches();

    return ORB_SLAM::tests::exitStatus();
}
