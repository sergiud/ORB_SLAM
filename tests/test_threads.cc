// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests of the synchronization of the worker threads of local mapping and
// loop closing with their clients.

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <format>
#include <future>
#include <memory>
#include <mutex>
#include <print>
#include <set>
#include <stop_token>
#include <string_view>
#include <thread>
#include <tuple>

#include <sys/resource.h>

#include <opencv2/core.hpp>

#include "Check.h"
#include "Frame.h"
#include "Frames.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapPoint.h"
#include "ORBVocabulary.h"

namespace {

using ORB_SLAM::tests::check;

using namespace std::chrono_literals;

// Generous enough that a loaded machine does not stall a correct worker
// past it, while a worker that ignores the request never meets it.
constexpr auto deadline = 10s;
constexpr auto pollInterval = 1ms;
// How long a paused worker is watched for resuming on its own.
constexpr auto pauseObservation = 50ms;
// Enough requests that one of them meets the worker resetting right after
// the request, before the client waits.
constexpr int resetRequests = 1000;

// Polls the predicate until it holds or the deadline passes, and returns
// whether it holds.
template<class Predicate>
bool waitUntil(Predicate predicate)
{
    const auto until = std::chrono::steady_clock::now() + deadline;

    bool holds = predicate();

    while (!holds && std::chrono::steady_clock::now() < until) {
        std::this_thread::sleep_for(pollInterval);
        holds = predicate();
    }

    return holds;
}

// Exposes the reset step of a worker loop, so that a test runs it in place
// of the worker thread.
template<class Worker>
class ResettingWorker : public Worker
{
public:
    using Worker::Worker;
    using Worker::ResetIfRequested;
};

// Exposes whether keyframes wait in the queue of local mapping.
class QueueingLocalMapping : public ORB_SLAM::LocalMapping
{
public:
    using ORB_SLAM::LocalMapping::LocalMapping;
    using ORB_SLAM::LocalMapping::CheckNewKeyFrames;
};

// Creates a keyframe at the origin from a frame without keypoints. Local
// mapping only queues it, so it needs neither image nor vocabulary.
std::unique_ptr<ORB_SLAM::KeyFrame> makeKeyFrame(ORB_SLAM::Map& map)
{
    ORB_SLAM::Frame frame =
        ORB_SLAM::tests::makeFrame(cv::Mat::eye(4, 4, CV_32F));

    return std::make_unique<ORB_SLAM::KeyFrame>(frame, &map, nullptr);
}

// Runs the worker in a thread, waits until it reaches the state the ready
// predicate describes, requests a stop and checks that the worker returns
// in time.
template<class Worker, class Ready>
void returnsOnStop(Worker& worker, Ready ready, std::string_view what)
{
    std::promise<void> finished;
    std::future<void> done = finished.get_future();

    std::jthread thread{[&worker, &finished](std::stop_token stopToken) {
        worker.Run(stopToken);
        finished.set_value();
    }};

    check(waitUntil(ready),
          std::format("{}: the worker reaches its state", what));

    thread.request_stop();

    if (done.wait_for(deadline) != std::future_status::ready) {
        std::println(stderr, "FAIL: {}: the worker still runs {} after the stop request",
                     what, deadline);
        // The thread cannot be joined, and the objects it refers to must
        // outlive it, so the test ends here.
        std::_Exit(EXIT_FAILURE);
    }
}

// A local mapping that waits for keyframes returns.
void localMappingReturnsWhenIdle()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    returnsOnStop(localMapping, [] { return true; }, "idle local mapping");
}

// A local mapping that loop closing has paused waits until it resumes and
// still returns.
void localMappingReturnsWhenPaused()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    localMapping.RequestPause();

    returnsOnStop(localMapping,
                  [&localMapping] { return localMapping.isPaused(); },
                  "paused local mapping");
}

// A local mapping that loop closing has paused accepts no keyframes before
// it resumes. The check can only miss a worker that resumes too early, it
// never fails a correct one.
void localMappingStaysPaused()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    localMapping.SetAcceptKeyFrames(false);
    localMapping.RequestPause();

    returnsOnStop(localMapping,
                  [&localMapping] {
                      if (!localMapping.isPaused()) {
                          return false;
                      }

                      std::this_thread::sleep_for(pauseObservation);
                      check(!localMapping.AcceptKeyFrames(),
                            "paused local mapping accepts no keyframes");
                      return true;
                  },
                  "paused local mapping before it resumes");
}

// A local mapping that loop closing resumed accepts keyframes again.
void localMappingAcceptsKeyFramesAfterResume()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    localMapping.SetAcceptKeyFrames(false);
    localMapping.RequestPause();

    std::jthread thread{[&localMapping](std::stop_token stopToken) {
        localMapping.Run(stopToken);
    }};

    check(waitUntil([&localMapping] { return localMapping.isPaused(); }),
          "local mapping pauses on request");

    localMapping.Resume();

    check(waitUntil(
              [&localMapping] { return localMapping.AcceptKeyFrames(); }),
          std::format("resumed local mapping accepts keyframes within {}",
                      deadline));
}

// A resume that comes before the worker pauses withdraws the request, so the
// worker keeps running instead of waiting for a second resume.
void localMappingPausesOnlyOnRequest()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    localMapping.RequestPause();
    localMapping.Resume();
    localMapping.Pause();

    check(!localMapping.isPaused(),
          "local mapping resumed before it pauses keeps running");
}

// Local mapping processes the keyframes it accepted before it pauses, so that
// none of them is left behind while loop closing changes the map.
void localMappingPausesWithEmptyQueue()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};
    const auto keyFrame = makeKeyFrame(map);

    check(localMapping.InsertKeyFrame(keyFrame.get()),
          "local mapping accepts a keyframe while it runs");
    localMapping.RequestPause();
    localMapping.Pause();

    check(!localMapping.isPaused(),
          "local mapping with a queued keyframe does not pause");
}

// Local mapping refuses keyframes once a pause is requested, which leaves them
// with the caller.
void localMappingRefusesKeyFramesWhenPausing()
{
    ORB_SLAM::Map map;
    QueueingLocalMapping localMapping{&map};
    auto keyFrame = makeKeyFrame(map);

    localMapping.RequestPause();

    if (localMapping.InsertKeyFrame(keyFrame.get())) {
        // Local mapping owns the keyframe it accepted.
        std::ignore = keyFrame.release();
        check(false, "local mapping refuses a keyframe while pausing");
    }

    check(!localMapping.CheckNewKeyFrames(),
          "local mapping queues no keyframe while pausing");
}

// A resume keeps the keyframes that local mapping accepted before the pause
// request, rather than destroying keyframes the tracker still refers to.
void localMappingKeepsKeyFramesOnResume()
{
    ORB_SLAM::Map map;
    QueueingLocalMapping localMapping{&map};
    auto keyFrame = makeKeyFrame(map);

    localMapping.InsertKeyFrame(keyFrame.get());
    localMapping.RequestPause();
    localMapping.Resume();

    const bool kept = localMapping.CheckNewKeyFrames();

    if (!kept) {
        // The resume destroyed the keyframe already.
        std::ignore = keyFrame.release();
    }

    check(kept, "local mapping keeps its queued keyframes on resume");
}

// Waiting until local mapping pauses returns only once the worker paused, and
// reports that it did.
void waitsUntilLocalMappingPauses()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    localMapping.RequestPause();

    std::atomic<bool> returned{false};
    std::atomic<bool> paused{false};

    std::jthread waiter{[&localMapping, &returned, &paused](std::stop_token stopToken) {
        paused = localMapping.WaitUntilPaused(stopToken);
        returned = true;
    }};

    // The check can only miss a wait that returns too early, it never fails
    // a correct one.
    std::this_thread::sleep_for(pauseObservation);
    check(!returned, "the wait returns only once local mapping paused");

    localMapping.Pause();

    check(waitUntil([&returned] { return returned.load(); }),
          std::format("the wait returns within {} once local mapping paused",
                      deadline));
    check(paused, "the wait reports that local mapping paused");
}

// Waiting until local mapping stops returns on a stop request of the waiting
// thread, and reports that local mapping did not pause.
void waitForLocalMappingEndsOnStopRequest()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    localMapping.RequestPause();

    std::atomic<bool> returned{false};
    std::atomic<bool> paused{true};

    std::jthread waiter{[&localMapping, &returned, &paused](std::stop_token stopToken) {
        paused = localMapping.WaitUntilPaused(stopToken);
        returned = true;
    }};

    waiter.request_stop();

    if (!waitUntil([&returned] { return returned.load(); })) {
        std::println(stderr,
                     "FAIL: the wait still runs {} after a stop request",
                     deadline);
        // The waiter cannot be joined, so the test ends here.
        std::_Exit(EXIT_FAILURE);
    }

    check(!paused, "the wait reports that local mapping did not pause");
}

// Exposes the loop correction of loop closing.
class CorrectingLoopClosing : public ORB_SLAM::LoopClosing
{
public:
    using ORB_SLAM::LoopClosing::LoopClosing;
    using ORB_SLAM::LoopClosing::CorrectLoop;
};

// Loop closing changes the map only once local mapping paused. Asked to end
// before that, it gives up the correction and resumes local mapping.
void loopCorrectionWaitsForLocalMapping()
{
    ORB_SLAM::Map map;
    ORB_SLAM::ORBVocabulary vocabulary;
    ORB_SLAM::KeyFrameDatabase database{vocabulary};
    ORB_SLAM::LocalMapping localMapping{&map};
    CorrectingLoopClosing loopClosing{&map, &database, &vocabulary};

    loopClosing.SetLocalMapper(&localMapping);

    std::stop_source stopSource;
    stopSource.request_stop();

    loopClosing.CorrectLoop(stopSource.get_token());

    check(!localMapping.pauseRequested(),
          "loop closing resumes local mapping when it gives up the "
          "correction");
}

// Tracking interrupts the bundle adjustment of local mapping while loop
// closing requests a pause, which ThreadSanitizer checks for data races.
void bundleAdjustmentInterruptedConcurrently()
{
    constexpr int requests = 1000;

    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    std::jthread tracking{[&localMapping] {
        for (int i = 0; i < requests; ++i) {
            localMapping.InterruptBA();
        }
    }};

    for (int i = 0; i < requests; ++i) {
        localMapping.RequestPause();
        localMapping.Resume();
    }
}

// Exposes the loop candidates that loop closing keeps between keyframes.
class CandidateLoopClosing : public ORB_SLAM::LoopClosing
{
public:
    using ORB_SLAM::LoopClosing::LoopClosing;
    using ORB_SLAM::LoopClosing::ResetIfRequested;
    using ORB_SLAM::LoopClosing::mbResetRequested;
    using ORB_SLAM::LoopClosing::mpMatchedKF;
    using ORB_SLAM::LoopClosing::mvConsistentGroups;
    using ORB_SLAM::LoopClosing::mvpEnoughConsistentCandidates;
    using ORB_SLAM::LoopClosing::mpCurrentKF;
    using ORB_SLAM::LoopClosing::mvpCurrentConnectedKFs;
    using ORB_SLAM::LoopClosing::mvpCurrentMatchedPoints;
    using ORB_SLAM::LoopClosing::mvpLoopMapPoints;
};

// A reset destroys every keyframe, so loop closing keeps no loop candidate
// across it, which a new keyframe at the same address would match.
void loopClosingForgetsCandidatesOnReset()
{
    ORB_SLAM::Map map;
    ORB_SLAM::ORBVocabulary vocabulary;
    ORB_SLAM::KeyFrameDatabase database{vocabulary};
    CandidateLoopClosing loopClosing{&map, &database, &vocabulary};
    const auto keyFrame = makeKeyFrame(map);

    loopClosing.mvConsistentGroups.emplace_back(
        std::set<ORB_SLAM::KeyFrame*>{keyFrame.get()}, 1);
    loopClosing.mvpEnoughConsistentCandidates.push_back(keyFrame.get());
    loopClosing.mpMatchedKF = keyFrame.get();
    loopClosing.mbResetRequested = true;

    loopClosing.ResetIfRequested();

    check(loopClosing.mvConsistentGroups.empty() &&
              loopClosing.mvpEnoughConsistentCandidates.empty() &&
              loopClosing.mpMatchedKF == nullptr,
          std::format("loop closing keeps no candidate after a reset, got {} "
                      "groups, {} candidates and {} matched keyframe",
                      loopClosing.mvConsistentGroups.size(),
                      loopClosing.mvpEnoughConsistentCandidates.size(),
                      loopClosing.mpMatchedKF ? "a" : "no"));
}

// Loop closing keeps no keyframe or map point across a reset, which destroys
// them all.
void loopClosingForgetsLoopOnReset()
{
    ORB_SLAM::Map map;
    ORB_SLAM::ORBVocabulary vocabulary;
    ORB_SLAM::KeyFrameDatabase database{vocabulary};
    CandidateLoopClosing loopClosing{&map, &database, &vocabulary};
    const auto keyFrame = makeKeyFrame(map);
    const auto point = std::make_unique<ORB_SLAM::MapPoint>(
        cv::Mat::zeros(3, 1, CV_32F), keyFrame.get(), &map);

    loopClosing.mpCurrentKF = keyFrame.get();
    loopClosing.mvpCurrentConnectedKFs = {keyFrame.get()};
    loopClosing.mvpCurrentMatchedPoints = {point.get()};
    loopClosing.mvpLoopMapPoints = {point.get()};
    loopClosing.mbResetRequested = true;

    loopClosing.ResetIfRequested();

    check(loopClosing.mpCurrentKF == nullptr,
          "loop closing keeps no current keyframe after a reset");
    check(loopClosing.mvpCurrentConnectedKFs.empty(),
          "loop closing keeps no connected keyframe after a reset");
    check(loopClosing.mvpCurrentMatchedPoints.empty(),
          "loop closing keeps no matched map point after a reset");
    check(loopClosing.mvpLoopMapPoints.empty(),
          "loop closing keeps no loop map point after a reset");
}

// Exposes the reset step of local mapping and its request.
class ResetLocalMapping : public ORB_SLAM::LocalMapping
{
public:
    using ORB_SLAM::LocalMapping::LocalMapping;
    using ORB_SLAM::LocalMapping::ResetIfRequested;
    using ORB_SLAM::LocalMapping::mbResetRequested;
    using ORB_SLAM::LocalMapping::mMutexReset;
    using ORB_SLAM::LocalMapping::mpCurrentKeyFrame;
};

// Local mapping keeps no current keyframe across a reset, which destroys it.
void localMappingForgetsKeyFrameOnReset()
{
    ORB_SLAM::Map map;
    ResetLocalMapping localMapping{&map};
    const auto keyFrame = makeKeyFrame(map);

    localMapping.mpCurrentKeyFrame = keyFrame.get();
    localMapping.mbResetRequested = true;

    localMapping.ResetIfRequested();

    check(localMapping.mpCurrentKeyFrame == nullptr,
          "local mapping keeps no current keyframe after a reset");
}

// Tracking inserts keyframes while local mapping resets, which
// ThreadSanitizer checks for data races.
void localMappingResetsWhileInserting()
{
    constexpr int resets = 1000;

    ORB_SLAM::Map map;
    ResetLocalMapping localMapping{&map};
    const auto keyFrame = makeKeyFrame(map);

    std::atomic<bool> done{false};

    std::jthread tracking{[&localMapping, &keyFrame, &done] {
        while (!done) {
            localMapping.InsertKeyFrame(keyFrame.get());
        }
    }};

    for (int i = 0; i < resets; ++i) {
        {
            std::scoped_lock lock{localMapping.mMutexReset};
            localMapping.mbResetRequested = true;
        }
        localMapping.ResetIfRequested();
    }

    done = true;
}

// Returns the voluntary context switches of the process so far.
long voluntarySwitches()
{
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    return usage.ru_nvcsw;
}

// Idle workers wait for work instead of waking up periodically to look for
// it, which shows in the context switches of the process.
void idleWorkersSleep()
{
    constexpr auto idle = 500ms;
    // Polling every few milliseconds switches hundreds of times in the idle
    // period, waiting only a few times.
    constexpr long maxSwitches = 50;

    ORB_SLAM::Map map;
    ORB_SLAM::ORBVocabulary vocabulary;
    ORB_SLAM::KeyFrameDatabase database{vocabulary};
    ORB_SLAM::LocalMapping localMapping{&map};
    ORB_SLAM::LoopClosing loopClosing{&map, &database, &vocabulary};

    std::jthread mapping{[&localMapping](std::stop_token stopToken) {
        localMapping.Run(stopToken);
    }};
    std::jthread closing{[&loopClosing](std::stop_token stopToken) {
        loopClosing.Run(stopToken);
    }};

    // Let the workers reach their wait first.
    std::this_thread::sleep_for(pauseObservation);

    const long before = voluntarySwitches();
    std::this_thread::sleep_for(idle);
    const long switches = voluntarySwitches() - before;

    check(switches <= maxSwitches,
          std::format("idle workers switch at most {} times in {}, got {}",
                      maxSwitches, idle, switches));
}

// Fails the test and ends it unless the future becomes ready in time, since
// the thread behind it cannot be joined then.
void requireReady(std::future<void>& future, std::string_view what)
{
    if (future.wait_for(deadline) != std::future_status::ready) {
        std::println(stderr, "FAIL: {} within {}", what, deadline);
        std::_Exit(EXIT_FAILURE);
    }
}

// A running local mapping that waits for keyframes pauses when loop closing
// asks it to.
void idleLocalMappingPauses()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    std::jthread mapping{[&localMapping](std::stop_token stopToken) {
        localMapping.Run(stopToken);
    }};

    // Let the worker reach its wait first.
    std::this_thread::sleep_for(pauseObservation);

    auto paused = std::async(std::launch::async, [&localMapping] {
        localMapping.RequestPause();
        std::ignore = localMapping.WaitUntilPaused(std::stop_token{});
    });

    requireReady(paused, "idle local mapping pauses on request");
    localMapping.Resume();
}

// Running workers that wait for keyframes reset when the tracker asks them
// to.
void idleWorkersReset()
{
    ORB_SLAM::Map map;
    ORB_SLAM::ORBVocabulary vocabulary;
    ORB_SLAM::KeyFrameDatabase database{vocabulary};
    ORB_SLAM::LocalMapping localMapping{&map};
    ORB_SLAM::LoopClosing loopClosing{&map, &database, &vocabulary};

    std::jthread mapping{[&localMapping](std::stop_token stopToken) {
        localMapping.Run(stopToken);
    }};
    std::jthread closing{[&loopClosing](std::stop_token stopToken) {
        loopClosing.Run(stopToken);
    }};

    // Let the workers reach their wait first.
    std::this_thread::sleep_for(pauseObservation);

    auto mappingReset = std::async(std::launch::async, [&localMapping] {
        localMapping.RequestReset();
    });
    requireReady(mappingReset, "idle local mapping resets on request");

    auto closingReset = std::async(std::launch::async, [&loopClosing] {
        loopClosing.RequestReset();
    });
    requireReady(closingReset, "idle loop closing resets on request");
}

// A running local mapping that waits for keyframes processes one as soon as
// it is inserted, after which it accepts keyframes again.
void idleLocalMappingProcessesKeyFrame()
{
    constexpr int branching = 2;
    constexpr int depth = 1;

    ORB_SLAM::ORBVocabulary vocabulary;
    vocabulary.create(
        {{cv::Mat::zeros(1, ORB_SLAM::tests::descriptorBytes, CV_8U),
          cv::Mat::ones(1, ORB_SLAM::tests::descriptorBytes, CV_8U)}},
        branching, depth);
    ORB_SLAM::KeyFrameDatabase database{vocabulary};
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    // Local mapping reads the scale factor between pyramid levels, so the
    // keyframe has the pyramid of the extractor.
    constexpr ORB_SLAM::tests::Pyramid pyramid{1.2F, 8};
    ORB_SLAM::Frame frame = ORB_SLAM::tests::makeFrame(
        cv::Mat::eye(4, 4, CV_32F), cv::Mat::eye(3, 3, CV_32F), {}, pyramid);
    frame.mpORBvocabulary = &vocabulary;
    // Not the first keyframe of the map, which local mapping never culls.
    ORB_SLAM::KeyFrame::nNextId = 1;
    auto keyFrame =
        std::make_unique<ORB_SLAM::KeyFrame>(frame, &map, &database);

    std::jthread mapping{[&localMapping](std::stop_token stopToken) {
        localMapping.Run(stopToken);
    }};

    // Let the worker reach its wait first.
    std::this_thread::sleep_for(pauseObservation);

    localMapping.InsertKeyFrame(keyFrame.get());

    const bool processed =
        waitUntil([&localMapping] { return localMapping.AcceptKeyFrames(); });

    if (processed) {
        // Local mapping added the keyframe to the map, which owns it.
        std::ignore = keyFrame.release();
    }

    check(processed,
          std::format("idle local mapping processes an inserted keyframe "
                      "within {}",
                      deadline));
}

// Exposes whether keyframes wait in the queue of loop closing.
class QueueingLoopClosing : public ORB_SLAM::LoopClosing
{
public:
    using ORB_SLAM::LoopClosing::LoopClosing;
    using ORB_SLAM::LoopClosing::CheckNewKeyFrames;
};

// A running loop closing that waits for keyframes takes one as soon as local
// mapping inserts it. A keyframe so soon after the start of the map only
// enters the database.
void idleLoopClosingTakesKeyFrame()
{
    ORB_SLAM::Map map;
    ORB_SLAM::ORBVocabulary vocabulary;
    ORB_SLAM::KeyFrameDatabase database{vocabulary};
    QueueingLoopClosing loopClosing{&map, &database, &vocabulary};

    ORB_SLAM::Frame frame =
        ORB_SLAM::tests::makeFrame(cv::Mat::eye(4, 4, CV_32F));
    // Loop closing ignores the first keyframe of the map.
    ORB_SLAM::KeyFrame::nNextId = 1;
    ORB_SLAM::KeyFrame keyFrame{frame, &map, &database};

    std::jthread closing{[&loopClosing](std::stop_token stopToken) {
        loopClosing.Run(stopToken);
    }};

    // Let the worker reach its wait first.
    std::this_thread::sleep_for(pauseObservation);

    loopClosing.InsertKeyFrame(&keyFrame);

    check(waitUntil([&loopClosing] { return !loopClosing.CheckNewKeyFrames(); }),
          std::format("idle loop closing takes an inserted keyframe within {}",
                      deadline));
}

// A reset request returns once the worker reset, also if the worker resets
// right after the request. The test resets in a tight loop in place of the
// worker thread, which makes that order likely.
template<class Worker>
void resetRequestReturns(Worker& worker, std::string_view what)
{
    for (int i = 0; i < resetRequests; ++i) {
        std::atomic<bool> returned{false};

        std::jthread requester{[&worker, &returned] {
            worker.RequestReset();
            returned = true;
        }};

        const auto until = std::chrono::steady_clock::now() + deadline;

        while (!returned && std::chrono::steady_clock::now() < until) {
            worker.ResetIfRequested();
        }

        if (!returned) {
            std::println(stderr,
                         "FAIL: {}: reset request {} still waits {} after "
                         "the request",
                         what, i, deadline);
            // The requester cannot be joined, so the test ends here.
            std::_Exit(EXIT_FAILURE);
        }
    }
}

void localMappingResetReturns()
{
    ORB_SLAM::Map map;
    ResettingWorker<ORB_SLAM::LocalMapping> localMapping{&map};

    resetRequestReturns(localMapping, "local mapping");
}

void loopClosingResetReturns()
{
    ORB_SLAM::Map map;
    ORB_SLAM::ORBVocabulary vocabulary;
    ORB_SLAM::KeyFrameDatabase database{vocabulary};
    ResettingWorker<ORB_SLAM::LoopClosing> loopClosing{&map, &database,
                                                       &vocabulary};

    resetRequestReturns(loopClosing, "loop closing");
}

// A loop closing that waits for keyframes returns.
void loopClosingReturnsWhenIdle()
{
    ORB_SLAM::Map map;
    ORB_SLAM::ORBVocabulary vocabulary;
    ORB_SLAM::KeyFrameDatabase database{vocabulary};
    ORB_SLAM::LoopClosing loopClosing{&map, &database, &vocabulary};

    returnsOnStop(loopClosing, [] { return true; }, "idle loop closing");
}

} // namespace

int main()
{
    localMappingReturnsWhenIdle();
    localMappingReturnsWhenPaused();
    localMappingStaysPaused();
    localMappingAcceptsKeyFramesAfterResume();
    localMappingPausesOnlyOnRequest();
    localMappingPausesWithEmptyQueue();
    localMappingRefusesKeyFramesWhenPausing();
    localMappingKeepsKeyFramesOnResume();
    waitsUntilLocalMappingPauses();
    waitForLocalMappingEndsOnStopRequest();
    loopCorrectionWaitsForLocalMapping();
    bundleAdjustmentInterruptedConcurrently();
    loopClosingForgetsCandidatesOnReset();
    loopClosingForgetsLoopOnReset();
    localMappingResetsWhileInserting();
    localMappingForgetsKeyFrameOnReset();
    idleWorkersSleep();
    idleLocalMappingPauses();
    idleWorkersReset();
    idleLocalMappingProcessesKeyFrame();
    idleLoopClosingTakesKeyFrame();
    loopClosingReturnsWhenIdle();
    localMappingResetReturns();
    loopClosingResetReturns();

    return ORB_SLAM::tests::exitStatus();
}
