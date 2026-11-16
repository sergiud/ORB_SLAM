// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests of the map initialization from two frames.

#include <format>
#include <numeric>
#include <vector>

#include <opencv2/core.hpp>

#include "Check.h"
#include "Frames.h"
#include "Initializer.h"

namespace {

using ORB_SLAM::tests::check;

constexpr float focalLength = 500;
constexpr float sigma = 1;
constexpr int iterations = 200;

cv::Mat cameraMatrix()
{
    return cv::Mat_<float>({3, 3}, {focalLength, 0, 320,
                                    0, focalLength, 240,
                                    0, 0, 1});
}

// Matches without any spread support neither a homography nor a fundamental
// matrix, so the initialization fails rather than reconstructing.
void degenerateMatchesDoNotInitialize()
{
    constexpr int matches = 20;

    const std::vector<cv::KeyPoint> keyPoints(matches, cv::KeyPoint{100, 100, 1});
    const ORB_SLAM::Frame reference = ORB_SLAM::tests::makeFrame(
        cv::Mat::eye(4, 4, CV_32F), cameraMatrix(), keyPoints);
    const ORB_SLAM::Frame current = ORB_SLAM::tests::makeFrame(
        cv::Mat::eye(4, 4, CV_32F), cameraMatrix(), keyPoints);

    std::vector<int> matched(matches);
    std::iota(matched.begin(), matched.end(), 0);

    ORB_SLAM::Initializer initializer{reference, sigma, iterations};
    cv::Mat rotation;
    cv::Mat translation;
    std::vector<cv::Point3f> points;
    std::vector<bool> triangulated;

    bool initialized = false;

    try {
        initialized = initializer.Initialize(current, matched, rotation,
                                             translation, points, triangulated);
    }
    catch (const cv::Exception& e) {
        check(false, std::format("degenerate matches fail the initialization "
                                 "without error, got {:?}",
                                 e.what()));
        return;
    }

    check(!initialized, "degenerate matches do not initialize the map");
}

} // namespace

int main()
{
    degenerateMatchesDoNotInitialize();

    return ORB_SLAM::tests::exitStatus();
}
