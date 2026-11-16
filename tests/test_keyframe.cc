// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests of the covisibility graph and spanning tree that keyframes form.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <format>
#include <future>
#include <memory>
#include <new>
#include <optional>
#include <print>
#include <thread>
#include <tuple>
#include <vector>

#include <opencv2/core.hpp>

#include "Check.h"
#include "Frames.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "Scene.h"
#include "Storage.h"

namespace {

using ORB_SLAM::tests::check;

using namespace std::chrono_literals;

// Generous enough that a loaded machine does not stall a correct erasure
// past it, while a deadlocked one never meets it.
constexpr auto deadline = 10s;
// Enough repetitions that the erasures interleave in the order that
// deadlocks.
constexpr int repetitions = 50;
// Children of each keyframe, which lengthen the critical section.
constexpr int children = 100;
constexpr int strongWeight = 100;
constexpr int parentWeight = 60;
constexpr int weakWeight = 50;
// Connections another thread adds while a keyframe is erased.
constexpr int connections = 2000;

ORB_SLAM::KeyFrame* addKeyFrame(ORB_SLAM::tests::Scene& scene)
{
    return scene.addKeyFrame(cv::Mat::eye(4, 4, CV_32F),
                             cv::Mat::eye(3, 3, CV_32F));
}

// Erasing a keyframe and its parent at the same time finishes. The children
// of the first keyframe are covisible with the parent, so that both
// erasures reassign children while the other runs.
void erasingNeighborsFinishes()
{
    for (int repetition = 0; repetition < repetitions; ++repetition) {
        ORB_SLAM::tests::Scene scene;
        // The first keyframe is never erased.
        addKeyFrame(scene);
        ORB_SLAM::KeyFrame* const root = addKeyFrame(scene);
        ORB_SLAM::KeyFrame* const parent = addKeyFrame(scene);
        ORB_SLAM::KeyFrame* const child = addKeyFrame(scene);

        parent->ChangeParent(root);
        child->ChangeParent(parent);
        child->AddConnection(parent, strongWeight);
        parent->AddConnection(child, strongWeight);

        for (int i = 0; i < children; ++i) {
            ORB_SLAM::KeyFrame* const grandchild = addKeyFrame(scene);
            grandchild->ChangeParent(child);
            grandchild->AddConnection(parent, weakWeight);
            grandchild->AddConnection(child, parentWeight);

            ORB_SLAM::KeyFrame* const sibling = addKeyFrame(scene);
            sibling->ChangeParent(parent);
            sibling->AddConnection(root, weakWeight);
        }

        std::atomic<bool> start{false};

        auto erase = [&start](ORB_SLAM::KeyFrame* keyFrame) {
            return std::async(std::launch::async, [&start, keyFrame] {
                while (!start) {
                    std::this_thread::yield();
                }
                keyFrame->SetBadFlag();
            });
        };

        std::future<void> first = erase(child);
        std::future<void> second = erase(parent);
        start = true;

        if (first.wait_for(deadline) != std::future_status::ready ||
            second.wait_for(deadline) != std::future_status::ready) {
            std::println(stderr,
                         "FAIL: erasing a keyframe and its parent at the same "
                         "time still runs {} later, in repetition {}",
                         deadline, repetition);
            // The erasing threads cannot be joined, so the test ends here.
            std::_Exit(EXIT_FAILURE);
        }
    }
}

// A keyframe keeps no connections once it is erased, also if another thread
// connects keyframes to it during the erasure.
void erasedKeyFrameKeepsNoConnections()
{
    ORB_SLAM::tests::Scene scene;
    addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const root = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const erased = addKeyFrame(scene);
    erased->ChangeParent(root);

    std::vector<ORB_SLAM::KeyFrame*> neighbors;

    for (int i = 0; i < connections; ++i) {
        neighbors.push_back(addKeyFrame(scene));
    }

    std::atomic<bool> start{false};

    std::jthread connecting{[&start, erased, &neighbors] {
        while (!start) {
            std::this_thread::yield();
        }

        for (ORB_SLAM::KeyFrame* neighbor : neighbors) {
            erased->AddConnection(neighbor, weakWeight);
        }
    }};

    start = true;
    erased->SetBadFlag();
    connecting.join();

    const std::size_t kept = erased->GetConnectedKeyFrames().size();

    check(kept == 0,
          std::format("an erased keyframe keeps no connections, got {}", kept));
}

// Updating the connections of an erased keyframe connects no other keyframe
// to it, also through the map points it observed.
void erasedKeyFrameStaysDisconnected()
{
    // A map point that keeps two observations or fewer is erased, so three
    // keyframes besides the erased one keep it alive.
    constexpr int observers = 3;

    ORB_SLAM::tests::Scene scene;
    addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const root = addKeyFrame(scene);
    const std::vector<cv::KeyPoint> keyPoint{cv::KeyPoint{0, 0, 1}};

    auto addObserver = [&scene, &keyPoint] {
        return scene.addKeyFrame(cv::Mat::eye(4, 4, CV_32F),
                                 cv::Mat::eye(3, 3, CV_32F), keyPoint);
    };

    ORB_SLAM::KeyFrame* const erased = addObserver();
    erased->ChangeParent(root);

    std::vector<ORB_SLAM::KeyFrame*> others;

    for (int i = 0; i < observers; ++i) {
        others.push_back(addObserver());
    }

    scene.addMapPoint(
        {0, 0, 1},
        {{erased, 0}, {others[0], 0}, {others[1], 0}, {others[2], 0}});

    erased->UpdateConnections();

    const bool connected = std::ranges::any_of(
        others, [erased](ORB_SLAM::KeyFrame* other) {
            return other->GetConnectedKeyFrames().contains(erased);
        });

    check(connected, "a keyframe that observes the same map point connects "
                     "to the keyframe before its erasure");

    erased->SetBadFlag();
    erased->UpdateConnections();
    others[0]->AddConnection(erased, weakWeight);

    check(erased->GetConnectedKeyFrames().empty(),
          "an erased keyframe connects to no keyframe");

    for (ORB_SLAM::KeyFrame* const other : others) {
        check(!other->GetConnectedKeyFrames().contains(erased),
              std::format("keyframe {} does not connect to an erased keyframe",
                          other->mnId));
    }
}

// Fusing map points replaces and erases the map points of a keyframe while
// tracking reads them, which ThreadSanitizer checks for data races.
void mapPointsReplacedConcurrently()
{
    constexpr int replacements = 1000;

    ORB_SLAM::tests::Scene scene;
    addKeyFrame(scene);
    const std::vector<cv::KeyPoint> keyPoint{cv::KeyPoint{0, 0, 1}};
    ORB_SLAM::KeyFrame* const keyFrame = scene.addKeyFrame(
        cv::Mat::eye(4, 4, CV_32F), cv::Mat::eye(3, 3, CV_32F), keyPoint);
    ORB_SLAM::MapPoint* const point =
        scene.addMapPoint({0, 0, 1}, {{keyFrame, 0}});

    std::atomic<bool> done{false};

    std::jthread tracking{[keyFrame, &done] {
        while (!done) {
            std::ignore = keyFrame->GetMapPoint(0);
        }
    }};

    for (int i = 0; i < replacements; ++i) {
        keyFrame->ReplaceMapPointMatch(0, point);
        keyFrame->EraseMapPointMatch(point);
    }

    done = true;
}

// Returns a vocabulary of a few words, which keyframes can share.
ORB_SLAM::ORBVocabulary wordsVocabulary()
{
    constexpr int branching = 3;
    constexpr int depth = 1;
    constexpr int descriptorBytes = ORB_SLAM::tests::descriptorBytes;

    std::vector<cv::Mat> descriptors;

    for (const unsigned char byte : {0x00, 0x0F, 0xFF}) {
        descriptors.push_back(
            cv::Mat(1, descriptorBytes, CV_8U, cv::Scalar::all(byte)));
    }

    ORB_SLAM::ORBVocabulary vocabulary;
    vocabulary.create({descriptors}, branching, depth);
    return vocabulary;
}

// Relocalisation ranks keyframes by their similarity to the frame, summed
// over covisible keyframes. A neighbor that shares too few words with the
// frame to be compared adds no similarity, also if it kept one from an
// earlier query.
void relocalisationIgnoresUncomparedNeighbors()
{
    constexpr float staleScore = 100;
    constexpr int weight = 50;
    constexpr DBoW2::WordId first = 1;
    constexpr DBoW2::WordId second = 2;
    constexpr long unsigned int frameId = 1000;

    ORB_SLAM::tests::Scene scene{wordsVocabulary()};
    addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const similar = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const neighbor = addKeyFrame(scene);

    similar->mBowVec.addWeight(first, 0.5);
    similar->mBowVec.addWeight(second, 0.5);
    neighbor->mBowVec.addWeight(first, 1.0);
    neighbor->mRelocScore = staleScore;
    similar->AddConnection(neighbor, weight);

    scene.database().add(similar);
    scene.database().add(neighbor);

    ORB_SLAM::Frame frame =
        ORB_SLAM::tests::makeFrame(cv::Mat::eye(4, 4, CV_32F));
    frame.mnId = frameId;
    frame.mBowVec = similar->mBowVec;

    const std::vector<ORB_SLAM::KeyFrame*> candidates =
        scene.database().DetectRelocalisationCandidates(&frame);

    check(candidates == std::vector<ORB_SLAM::KeyFrame*>{similar},
          std::format("the keyframe that shares all words is the only "
                      "candidate, got {} candidates, the first being the "
                      "{}",
                      candidates.size(),
                      candidates.empty() ? "none"
                      : candidates.front() == similar ? "similar keyframe"
                                                       : "neighbor"));
}

// Fusing map points compares these ids with the id of the current keyframe,
// so new keyframes and map points start with no keyframe, as do their scores
// of loop and relocalisation queries. Both are constructed in storage filled
// with a pattern other than zero, so that members the constructors leave
// uninitialized show.
void startsWithoutQueries()
{
    ORB_SLAM::tests::Scene scene;
    const ORB_SLAM::Frame frame =
        ORB_SLAM::tests::makeFrame(cv::Mat::eye(4, 4, CV_32F));

    alignas(ORB_SLAM::KeyFrame) std::byte keyFrameStorage[sizeof(ORB_SLAM::KeyFrame)];
    ORB_SLAM::tests::fillWithPattern(keyFrameStorage);
    ORB_SLAM::Frame copy{frame};
    auto* const keyFrame = new (keyFrameStorage)
        ORB_SLAM::KeyFrame{copy, &scene.map(), &scene.database()};

    check(keyFrame->mnFuseTargetForKF == 0 && keyFrame->mnLoopWords == 0 &&
              keyFrame->mLoopScore == 0 && keyFrame->mnRelocWords == 0 &&
              keyFrame->mRelocScore == 0,
          std::format("a keyframe starts without fusion target and query "
                      "scores, got target {}, loop words {}, loop score {}, "
                      "relocalisation words {}, relocalisation score {}",
                      keyFrame->mnFuseTargetForKF, keyFrame->mnLoopWords,
                      keyFrame->mLoopScore, keyFrame->mnRelocWords,
                      keyFrame->mRelocScore));

    alignas(ORB_SLAM::MapPoint) std::byte pointStorage[sizeof(ORB_SLAM::MapPoint)];
    ORB_SLAM::tests::fillWithPattern(pointStorage);
    auto* const point = new (pointStorage) ORB_SLAM::MapPoint{
        cv::Mat{cv::Vec3f{0, 0, 1}}, keyFrame, &scene.map()};

    check(point->mnFuseCandidateForKF == 0 && !point->mbTrackInView,
          std::format("a map point starts without fusion keyframe and outside "
                      "the view, got keyframe {} and in view {}",
                      point->mnFuseCandidateForKF, point->mbTrackInView));

    std::destroy_at(point);
    std::destroy_at(keyFrame);
}

// The median depth is that of the middle map point a keyframe sees, and a
// keyframe without map points has none.
void medianDepthOfMapPoints()
{
    ORB_SLAM::tests::Scene scene;
    addKeyFrame(scene);
    const std::vector<cv::KeyPoint> keyPoints(3, cv::KeyPoint{0, 0, 1});
    ORB_SLAM::KeyFrame* const empty = scene.addKeyFrame(
        cv::Mat::eye(4, 4, CV_32F), cv::Mat::eye(3, 3, CV_32F), keyPoints);
    ORB_SLAM::KeyFrame* const seeing = scene.addKeyFrame(
        cv::Mat::eye(4, 4, CV_32F), cv::Mat::eye(3, 3, CV_32F), keyPoints);

    for (std::size_t i = 0; i < keyPoints.size(); ++i) {
        scene.addMapPoint({0, 0, static_cast<float>(i + 1)}, {{seeing, i}});
    }

    const std::optional<float> none = empty->ComputeSceneMedianDepth();
    const std::optional<float> median = seeing->ComputeSceneMedianDepth();

    check(!none.has_value(),
          std::format("a keyframe without map points has no median depth, got "
                      "{}",
                      none.value_or(0)));
    check(median == 2.0F, std::format("the median depth is 2, got {}",
                                      median.value_or(0)));
}

// Loop closing releases a keyframe for erasure while local mapping tries to
// erase it, which ThreadSanitizer checks for data races. A loop edge keeps
// the keyframe from erasure, so that both keep trying.
void erasureReleasedConcurrently()
{
    constexpr int attempts = 1000;

    ORB_SLAM::tests::Scene scene;
    addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const root = addKeyFrame(scene);
    ORB_SLAM::KeyFrame* const keyFrame = addKeyFrame(scene);
    keyFrame->ChangeParent(root);
    keyFrame->AddLoopEdge(root);

    std::atomic<bool> done{false};

    std::jthread localMapping{[keyFrame, &done] {
        while (!done) {
            keyFrame->SetBadFlag();
        }
    }};

    for (int i = 0; i < attempts; ++i) {
        keyFrame->SetErase();
    }

    done = true;
}

// Tracking resets the map while the publishers read it, which
// ThreadSanitizer checks for data races.
void mapClearedConcurrently()
{
    constexpr int resets = 200;
    constexpr int keyFrames = 10;

    ORB_SLAM::Map map;
    ORB_SLAM::ORBVocabulary vocabulary;
    ORB_SLAM::KeyFrameDatabase database{vocabulary};
    ORB_SLAM::Frame frame =
        ORB_SLAM::tests::makeFrame(cv::Mat::eye(4, 4, CV_32F));

    std::atomic<bool> done{false};

    std::jthread publisher{[&map, &done] {
        while (!done) {
            std::ignore = map.GetAllKeyFrames();
        }
    }};

    for (int i = 0; i < resets; ++i) {
        for (int j = 0; j < keyFrames; ++j) {
            // The map owns the keyframe and deletes it on clearing.
            map.AddKeyFrame(new ORB_SLAM::KeyFrame{frame, &map, &database});
        }

        map.clear();
    }

    done = true;
}

} // namespace

int main()
{
    erasingNeighborsFinishes();
    erasedKeyFrameKeepsNoConnections();
    erasedKeyFrameStaysDisconnected();
    mapPointsReplacedConcurrently();
    relocalisationIgnoresUncomparedNeighbors();
    startsWithoutQueries();
    medianDepthOfMapPoints();
    erasureReleasedConcurrently();
    mapClearedConcurrently();

    return ORB_SLAM::tests::exitStatus();
}
