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
#include <print>
#include <stop_token>
#include <string_view>
#include <thread>

#include "Check.h"
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
    loopClosingReturnsWhenIdle();
    localMappingResetReturns();
    loopClosingResetReturns();

    return ORB_SLAM::tests::exitStatus();
}
