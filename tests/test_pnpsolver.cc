// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests of the camera pose RANSAC that relocalisation runs.

#include <cstddef>
#include <format>
#include <random>
#include <thread>
#include <tuple>
#include <vector>

#include <opencv2/core.hpp>

#include "Check.h"
#include "Frames.h"
#include "PnPsolver.h"
#include "Scene.h"

namespace {

using ORB_SLAM::tests::check;

constexpr int width = 640;
constexpr int height = 480;
constexpr float focalLength = 500;
constexpr std::size_t correspondences = 100;

cv::Mat cameraMatrix()
{
    return cv::Mat_<float>({3, 3}, {focalLength, 0, width / 2.0F,
                                    0, focalLength, height / 2.0F,
                                    0, 0, 1});
}

// Random points and keypoints, which no camera pose relates, with a frame
// that matches them.
struct RandomCorrespondences
{
    RandomCorrespondences()
    {
        ORB_SLAM::tests::setCamera(cameraMatrix(), width, height);

        std::mt19937 generator;
        std::uniform_real_distribution<float> coordinate{-1, 1};
        std::uniform_real_distribution<float> column{0, width};
        std::uniform_real_distribution<float> row{0, height};

        std::vector<cv::KeyPoint> keyPoints;

        for (std::size_t i = 0; i < correspondences; ++i) {
            keyPoints.emplace_back(column(generator), row(generator), 1.0F);
        }

        ORB_SLAM::KeyFrame* const keyFrame = scene.addKeyFrame(
            cv::Mat::eye(4, 4, CV_32F), cameraMatrix(), keyPoints);
        frame = ORB_SLAM::tests::makeFrame(cv::Mat::eye(4, 4, CV_32F),
                                           cameraMatrix(), keyPoints);

        for (std::size_t i = 0; i < correspondences; ++i) {
            matches.push_back(scene.addMapPoint(
                {coordinate(generator), coordinate(generator),
                 2 + coordinate(generator)},
                {{keyFrame, i}}));
        }
    }

    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::Frame frame;
    std::vector<ORB_SLAM::MapPoint*> matches;
};

constexpr double probability = 0.99;
constexpr int minInliers = 10;
constexpr int maxIterations = 300;
constexpr int sampleSize = 4;
constexpr float inlierRatio = 0.5F;
constexpr float chiSquare = 5.991F;

// Relocalisation interleaves the RANSAC of its candidate keyframes a few
// iterations at a time. Each call iterates as often as asked until the
// iterations that the 4 points of a sample need are spent.
void iteratesWithinBudget()
{
    constexpr int perCall = 5;
    // 72 iterations draw a sample of 4 inliers with probability 0.99, which
    // takes 15 calls of 5 iterations.
    constexpr int calls = 15;

    RandomCorrespondences data;
    ORB_SLAM::PnPsolver solver{data.frame, data.matches};
    solver.SetRansacParameters(probability, minInliers, maxIterations,
                               sampleSize, inlierRatio, chiSquare);

    bool noMore = false;
    int call = 0;

    while (!noMore && call <= calls) {
        std::vector<bool> inliers;
        int inlierCount = 0;
        solver.iterate(perCall, noMore, inliers, inlierCount);
        ++call;
    }

    check(call == calls,
          std::format("the iterations are spent after {} calls of {}, got {}",
                      calls, perCall, call));
}

// Solvers of different candidates may run in different threads, which
// ThreadSanitizer checks for data races.
void solversRunConcurrently()
{
    RandomCorrespondences first;
    RandomCorrespondences second;

    auto solve = [](const RandomCorrespondences& data) {
        ORB_SLAM::PnPsolver solver{data.frame, data.matches};
        solver.SetRansacParameters(probability, minInliers, maxIterations,
                                   sampleSize, inlierRatio, chiSquare);
        std::vector<bool> inliers;
        int inlierCount = 0;
        std::ignore = solver.find(inliers, inlierCount);
    };

    std::jthread other{[&solve, &second] { solve(second); }};
    solve(first);
}

} // namespace

int main()
{
    iteratesWithinBudget();
    solversRunConcurrently();

    return ORB_SLAM::tests::exitStatus();
}
