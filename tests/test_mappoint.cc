// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests of the map points that keyframes observe.

#include <cmath>
#include <format>
#include <string_view>
#include <vector>

#include <opencv2/core.hpp>

#include "Check.h"
#include "Frames.h"
#include "MapPoint.h"
#include "Scene.h"

namespace {

using ORB_SLAM::tests::check;

constexpr ORB_SLAM::tests::Pyramid pyramid{1.2F, 8};
constexpr float relativeTolerance = 1e-5F;
// A fraction of a scale step that rounds to the level next to it.
constexpr float partialStep = 0.3F;

bool near(float actual, float expected)
{
    return std::abs(actual - expected) <= relativeTolerance * std::abs(expected);
}

// The extractor detects a feature at a finer level the farther the camera is
// from it. A point first detected at a level from some distance stays
// detectable from where it appears at the coarsest level out to where it
// appears at the finest level.
void scaleInvarianceFollowsPyramid()
{
    constexpr int level = 2;
    constexpr float distance = 10;

    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const keyFrame = scene.addKeyFrame(
        cv::Mat::eye(4, 4, CV_32F), cv::Mat::eye(3, 3, CV_32F),
        {cv::KeyPoint{0, 0, 1, -1, 0, level}}, pyramid);
    ORB_SLAM::MapPoint* const point =
        scene.addMapPoint({0, 0, distance}, {{keyFrame, 0}});

    point->UpdateNormalAndDepth();

    const float finest = distance * std::pow(pyramid.scaleFactor, level);
    const float coarsest =
        finest / std::pow(pyramid.scaleFactor, pyramid.levels - 1);

    check(near(point->GetMaxDistanceInvariance(), finest),
          std::format("the point is detectable up to {}, got {}", finest,
                      point->GetMaxDistanceInvariance()));
    check(near(point->GetMinDistanceInvariance(), coarsest),
          std::format("the point is detectable from {}, got {}", coarsest,
                      point->GetMinDistanceInvariance()));

    const std::vector<float> scaleFactors = keyFrame->GetScaleFactors();

    const auto predicts = [point, &scaleFactors](float at, int expected,
                                                 std::string_view where) {
        const int predicted = point->PredictScale(at, scaleFactors);
        check(predicted == expected,
              std::format("{} the point is predicted at level {}, got {}",
                          where, expected, predicted));
    };

    predicts(distance, level, "from its first distance");
    predicts(distance / pyramid.scaleFactor, level + 1,
             "one scale step closer");
    predicts(distance * pyramid.scaleFactor, level - 1,
             "one scale step farther");
    predicts(finest, 0, "at its largest distance");
    predicts(coarsest, pyramid.levels - 1, "at its smallest distance");
    predicts(distance / std::pow(pyramid.scaleFactor, partialStep), level,
             "a fraction of a scale step closer");
    predicts(finest * pyramid.scaleFactor, 0, "beyond its largest distance");
    predicts(coarsest / pyramid.scaleFactor, pyramid.levels - 1,
             "within its smallest distance");

    const int single = point->PredictScale(distance, {1.0F});
    check(single == 0,
          std::format("a camera with one level predicts level 0, got {}",
                      single));
}

// The normal of a point is the unit direction from which the keyframes see
// it, which the tracker compares with a viewing direction as a cosine.
void normalIsUnitLength()
{
    constexpr float spread = 4;
    constexpr float distance = 4;

    ORB_SLAM::tests::Scene scene;
    const std::vector<cv::KeyPoint> keyPoint{cv::KeyPoint{0, 0, 1}};
    std::vector<ORB_SLAM::KeyFrame*> keyFrames;

    for (const float x : {-spread, spread}) {
        cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
        Tcw.at<float>(0, 3) = -x;
        keyFrames.push_back(scene.addKeyFrame(Tcw, cv::Mat::eye(3, 3, CV_32F),
                                              keyPoint, pyramid));
    }

    ORB_SLAM::MapPoint* const point = scene.addMapPoint(
        {0, 0, distance}, {{keyFrames[0], 0}, {keyFrames[1], 0}});
    point->UpdateNormalAndDepth();

    const double length = cv::norm(point->GetNormal());

    check(near(static_cast<float>(length), 1),
          std::format("the normal has unit length, got {}", length));
}

// Keyframes that see a point from opposite sides give it no direction, so
// its normal stays finite rather than dividing by zero.
void opposedViewsKeepNormalFinite()
{
    constexpr float distance = 4;

    ORB_SLAM::tests::Scene scene;
    const std::vector<cv::KeyPoint> keyPoint{cv::KeyPoint{0, 0, 1}};
    std::vector<ORB_SLAM::KeyFrame*> keyFrames;

    for (const float z : {-distance, distance}) {
        cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
        Tcw.at<float>(2, 3) = -z;
        keyFrames.push_back(scene.addKeyFrame(Tcw, cv::Mat::eye(3, 3, CV_32F),
                                              keyPoint, pyramid));
    }

    ORB_SLAM::MapPoint* const point = scene.addMapPoint(
        {0, 0, 0}, {{keyFrames[0], 0}, {keyFrames[1], 0}});
    point->UpdateNormalAndDepth();

    check(cv::checkRange(point->GetNormal()),
          "the normal of a point seen from opposite sides is finite");
}

} // namespace

int main()
{
    scaleInvarianceFollowsPyramid();
    normalIsUnitLength();
    opposedViewsKeepNormalFinite();

    return ORB_SLAM::tests::exitStatus();
}
