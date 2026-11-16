// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests of the tracking front end that need no images.

#include <cstddef>
#include <format>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <opencv2/core.hpp>

#include "Check.h"
#include "MapPoint.h"
#include "Tracking.h"

namespace {

using ORB_SLAM::tests::check;

constexpr double framesPerSecond = 30;
constexpr int features = 1000;
constexpr float scaleFactor = 1.2F;
constexpr int levels = 8;
constexpr int fastThreshold = 20;
constexpr int harrisScore = 1;

// Redirects std::clog into a string while it lives.
class ClogCapture
{
public:
    ClogCapture()
        : previous_{std::clog.rdbuf(text_.rdbuf())}
    {
    }

    ClogCapture(const ClogCapture&) = delete;
    ClogCapture& operator=(const ClogCapture&) = delete;

    ~ClogCapture()
    {
        std::clog.rdbuf(previous_);
    }

    [[nodiscard]] std::string text() const
    {
        return text_.str();
    }

private:
    std::ostringstream text_;
    std::streambuf* previous_;
};

// Creates a tracker for the camera matrix without vocabulary, publishers
// and map, which construction does not use.
ORB_SLAM::Tracking makeTracking(const cv::Mat& K, bool useMotionModel)
{
    return ORB_SLAM::Tracking{nullptr, nullptr, nullptr, nullptr, K,
                              cv::Mat{}, framesPerSecond, features,
                              scaleFactor, levels, fastThreshold,
                              harrisScore, useMotionModel};
}

// Construction logs whether the motion model is used, one plain line each.
void logsMotionModel(bool useMotionModel, std::string_view expected)
{
    std::string text;
    {
        ClogCapture capture;
        makeTracking(cv::Mat1f::eye(3, 3), useMotionModel);
        text = capture.text();
    }

    check(text == expected,
          std::format("the log reads {:?}, got {:?}", expected, text));
}

// A camera matrix that is not 3x3 is rejected with its size.
void rejectsCameraMatrix(const cv::Mat& K, std::string_view expected)
{
    std::string message;

    try {
        makeTracking(K, true);
    }
    catch (const std::invalid_argument& e) {
        message = e.what();
    }

    check(message == expected,
          std::format("the error reads {:?}, got {:?}", expected, message));
}

// Only the keypoints of the current frame that see a map point and are no
// outliers are tracked.
void keyPointsOfInliers()
{
    ORB_SLAM::Tracking tracking = makeTracking(cv::Mat1f::eye(3, 3), true);

    // The tracker only compares the map points with null, so storage of the
    // right size stands in for a constructed map point.
    alignas(ORB_SLAM::MapPoint) std::byte storage[sizeof(ORB_SLAM::MapPoint)];
    auto* const point = reinterpret_cast<ORB_SLAM::MapPoint*>(storage);

    ORB_SLAM::Frame& frame = tracking.mCurrentFrame;
    frame.mvKeys = {cv::KeyPoint{1, 1, 1}, cv::KeyPoint{2, 2, 1},
                    cv::KeyPoint{3, 3, 1}, cv::KeyPoint{4, 4, 1}};
    frame.mvpMapPoints = {point, nullptr, point, point};
    frame.mvbOutlier = {false, false, true, false};

    const std::vector<cv::KeyPoint> keyPoints = tracking.GetKeyPoints();

    check(keyPoints.size() == 2,
          std::format("two keypoints are tracked, got {}", keyPoints.size()));

    if (keyPoints.size() == 2) {
        check(keyPoints[0].pt == cv::Point2f(1, 1) &&
                  keyPoints[1].pt == cv::Point2f(4, 4),
              std::format("the keypoints at (1, 1) and (4, 4) are tracked, got "
                          "({}, {}) and ({}, {})",
                          keyPoints[0].pt.x, keyPoints[0].pt.y,
                          keyPoints[1].pt.x, keyPoints[1].pt.y));
    }
}

} // namespace

int main()
{
    logsMotionModel(true, "motion model: enabled\n");
    logsMotionModel(false, "motion model: disabled (not recommended)\n");
    rejectsCameraMatrix(cv::Mat1f::eye(2, 3),
                        "camera intrinsic matrix must be 3x3, got 2x3");
    rejectsCameraMatrix(cv::Mat{}, "camera intrinsic matrix must be 3x3, got 0x0");
    keyPointsOfInliers();

    return ORB_SLAM::tests::exitStatus();
}
