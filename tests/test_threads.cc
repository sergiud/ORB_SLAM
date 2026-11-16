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

// A local mapping that loop closing has paused waits for its release and
// still returns.
void localMappingReturnsWhenPaused()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    localMapping.RequestStop();

    returnsOnStop(localMapping,
                  [&localMapping] { return localMapping.isStopped(); },
                  "paused local mapping");
}

// A local mapping that loop closing has paused accepts no keyframes before
// its release. The check can only miss a worker that resumes too early, it
// never fails a correct one.
void localMappingStaysPaused()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    localMapping.SetAcceptKeyFrames(false);
    localMapping.RequestStop();

    returnsOnStop(localMapping,
                  [&localMapping] {
                      if (!localMapping.isStopped()) {
                          return false;
                      }

                      std::this_thread::sleep_for(pauseObservation);
                      check(!localMapping.AcceptKeyFrames(),
                            "paused local mapping accepts no keyframes");
                      return true;
                  },
                  "paused local mapping before its release");
}

// A local mapping that loop closing released accepts keyframes again.
void localMappingResumesAfterRelease()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    localMapping.SetAcceptKeyFrames(false);
    localMapping.RequestStop();

    std::jthread thread{[&localMapping](std::stop_token stopToken) {
        localMapping.Run(stopToken);
    }};

    check(waitUntil([&localMapping] { return localMapping.isStopped(); }),
          "local mapping pauses on request");

    localMapping.Release();

    check(waitUntil(
              [&localMapping] { return localMapping.AcceptKeyFrames(); }),
          std::format("released local mapping accepts keyframes within {}",
                      deadline));
}

// A release that comes before the worker stops withdraws the request, so the
// worker keeps running instead of waiting for a second release.
void localMappingStopsOnlyOnRequest()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    localMapping.RequestStop();
    localMapping.Release();
    localMapping.Stop();

    check(!localMapping.isStopped(),
          "local mapping released before it stops keeps running");
}

// Local mapping processes the keyframes it accepted before it stops, so that
// none of them is left behind while loop closing changes the map.
void localMappingStopsWithEmptyQueue()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};
    const auto keyFrame = makeKeyFrame(map);

    check(localMapping.InsertKeyFrame(keyFrame.get()),
          "local mapping accepts a keyframe while it runs");
    localMapping.RequestStop();
    localMapping.Stop();

    check(!localMapping.isStopped(),
          "local mapping with a queued keyframe does not stop");
}

// Local mapping refuses keyframes once a stop is requested, which leaves them
// with the caller.
void localMappingRefusesKeyFramesWhenStopping()
{
    ORB_SLAM::Map map;
    QueueingLocalMapping localMapping{&map};
    auto keyFrame = makeKeyFrame(map);

    localMapping.RequestStop();

    if (localMapping.InsertKeyFrame(keyFrame.get())) {
        // Local mapping owns the keyframe it accepted.
        std::ignore = keyFrame.release();
        check(false, "local mapping refuses a keyframe while stopping");
    }

    check(!localMapping.CheckNewKeyFrames(),
          "local mapping queues no keyframe while stopping");
}

// A release keeps the keyframes that local mapping accepted before the stop
// request, rather than destroying keyframes the tracker still refers to.
void localMappingKeepsKeyFramesOnRelease()
{
    ORB_SLAM::Map map;
    QueueingLocalMapping localMapping{&map};
    auto keyFrame = makeKeyFrame(map);

    localMapping.InsertKeyFrame(keyFrame.get());
    localMapping.RequestStop();
    localMapping.Release();

    const bool kept = localMapping.CheckNewKeyFrames();

    if (!kept) {
        // The release destroyed the keyframe already.
        std::ignore = keyFrame.release();
    }

    check(kept, "local mapping keeps its queued keyframes on release");
}

// Waiting until local mapping stops returns only once the worker stopped, and
// reports that it did.
void waitsUntilLocalMappingStops()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    localMapping.RequestStop();

    std::atomic<bool> returned{false};
    std::atomic<bool> stopped{false};

    std::jthread waiter{[&localMapping, &returned, &stopped](std::stop_token stopToken) {
        stopped = localMapping.WaitUntilStopped(stopToken);
        returned = true;
    }};

    // The check can only miss a wait that returns too early, it never fails
    // a correct one.
    std::this_thread::sleep_for(pauseObservation);
    check(!returned, "the wait returns only once local mapping stopped");

    localMapping.Stop();

    check(waitUntil([&returned] { return returned.load(); }),
          std::format("the wait returns within {} once local mapping stopped",
                      deadline));
    check(stopped, "the wait reports that local mapping stopped");
}

// Waiting until local mapping stops returns on a stop request of the waiting
// thread, and reports that local mapping did not stop.
void waitForLocalMappingEndsOnStopRequest()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    localMapping.RequestStop();

    std::atomic<bool> returned{false};
    std::atomic<bool> stopped{true};

    std::jthread waiter{[&localMapping, &returned, &stopped](std::stop_token stopToken) {
        stopped = localMapping.WaitUntilStopped(stopToken);
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

    check(!stopped, "the wait reports that local mapping did not stop");
}

// Exposes the loop correction of loop closing.
class CorrectingLoopClosing : public ORB_SLAM::LoopClosing
{
public:
    using ORB_SLAM::LoopClosing::LoopClosing;
    using ORB_SLAM::LoopClosing::CorrectLoop;
};

// Loop closing changes the map only once local mapping stopped. Asked to end
// before that, it gives up the correction and releases local mapping.
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

    check(!localMapping.stopRequested(),
          "loop closing releases local mapping when it gives up the "
          "correction");
}

// Tracking interrupts the bundle adjustment of local mapping while loop
// closing requests a stop, which ThreadSanitizer checks for data races.
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
        localMapping.RequestStop();
        localMapping.Release();
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

// Exposes the reset step of local mapping and its request.
class ResetLocalMapping : public ORB_SLAM::LocalMapping
{
public:
    using ORB_SLAM::LocalMapping::LocalMapping;
    using ORB_SLAM::LocalMapping::ResetIfRequested;
    using ORB_SLAM::LocalMapping::mbResetRequested;
    using ORB_SLAM::LocalMapping::mMutexReset;
};

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

// A running local mapping that waits for keyframes stops when loop closing
// asks it to.
void idleLocalMappingStops()
{
    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};

    std::jthread mapping{[&localMapping](std::stop_token stopToken) {
        localMapping.Run(stopToken);
    }};

    // Let the worker reach its wait first.
    std::this_thread::sleep_for(pauseObservation);

    auto stopped = std::async(std::launch::async, [&localMapping] {
        localMapping.RequestStop();
        std::ignore = localMapping.WaitUntilStopped(std::stop_token{});
    });

    requireReady(stopped, "idle local mapping stops on request");
    localMapping.Release();
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
    ORB_SLAM::KeyFrame keyFrame{frame, &map, &database};

    std::jthread mapping{[&localMapping](std::stop_token stopToken) {
        localMapping.Run(stopToken);
    }};

    // Let the worker reach its wait first.
    std::this_thread::sleep_for(pauseObservation);

    localMapping.InsertKeyFrame(&keyFrame);

    check(waitUntil([&localMapping] { return localMapping.AcceptKeyFrames(); }),
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
    localMappingResumesAfterRelease();
    localMappingStopsOnlyOnRequest();
    localMappingStopsWithEmptyQueue();
    localMappingRefusesKeyFramesWhenStopping();
    localMappingKeepsKeyFramesOnRelease();
    waitsUntilLocalMappingStops();
    waitForLocalMappingEndsOnStopRequest();
    loopCorrectionWaitsForLocalMapping();
    bundleAdjustmentInterruptedConcurrently();
    loopClosingForgetsCandidatesOnReset();
    localMappingResetsWhileInserting();
    idleWorkersSleep();
    idleLocalMappingStops();
    idleWorkersReset();
    idleLocalMappingProcessesKeyFrame();
    idleLoopClosingTakesKeyFrame();
    loopClosingReturnsWhenIdle();
    localMappingResetReturns();
    loopClosingResetReturns();

    return ORB_SLAM::tests::exitStatus();
}
