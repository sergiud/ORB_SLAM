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

} // namespace

int main()
{
    erasingNeighborsFinishes();
    erasedKeyFrameKeepsNoConnections();
    erasedKeyFrameStaysDisconnected();

    return ORB_SLAM::tests::exitStatus();
}
