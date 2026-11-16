// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests of the optimizations that refine poses and similarities.

#include <array>
#include <atomic>
#include <cstddef>
#include <format>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "Check.h"
#include "Frames.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Optimizer.h"
#include "Scene.h"

namespace {

using ORB_SLAM::tests::check;

constexpr float focalLength = 500;
constexpr float principalPointX = 320;
constexpr float principalPointY = 240;
constexpr int imageWidth = 640;
constexpr int imageHeight = 480;
constexpr ORB_SLAM::tests::Pyramid pyramid{1.2F, 8};
// Chi-square threshold of loop closing for two degrees of freedom.
constexpr float chiSquareThreshold = 10;

cv::Mat cameraMatrix()
{
    return cv::Mat_<float>({3, 3}, {focalLength, 0, principalPointX,
                                    0, focalLength, principalPointY,
                                    0, 0, 1});
}

// Returns the pose of a camera centered at x on the x-axis, looking along z.
cv::Mat poseAt(float x)
{
    cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
    Tcw.at<float>(0, 3) = -x;
    return Tcw;
}

// Projects the world point into the camera centered at x.
cv::Point2f project(const cv::Point3f& point, float x)
{
    return {focalLength * (point.x - x) / point.z + principalPointX,
            focalLength * point.y / point.z + principalPointY};
}

// Points in front of the cameras on the x-axis, at varying depths.
std::array<cv::Point3f, 12> scenePoints()
{
    return {cv::Point3f{-1.0F, -0.5F, 4.0F}, cv::Point3f{-0.5F, -0.5F, 4.5F},
            cv::Point3f{0.0F, -0.5F, 5.0F},  cv::Point3f{0.5F, -0.5F, 4.2F},
            cv::Point3f{1.0F, -0.5F, 4.8F},  cv::Point3f{1.5F, -0.5F, 5.5F},
            cv::Point3f{-1.0F, 0.5F, 5.2F},  cv::Point3f{-0.5F, 0.5F, 4.4F},
            cv::Point3f{0.0F, 0.5F, 4.6F},   cv::Point3f{0.5F, 0.5F, 5.4F},
            cv::Point3f{1.0F, 0.5F, 4.1F},   cv::Point3f{1.5F, 0.5F, 4.9F}};
}

// Two keyframes that observe the scene points exactly, with the pose of the
// second keyframe off along the x-axis. The first keyframe is the first of
// its map, whose pose the optimizations keep fixed.
struct PerturbedPair
{
    explicit PerturbedPair(float offset)
    {
        constexpr float secondCenter = 0.5F;

        ORB_SLAM::KeyFrame::nNextId = 0;
        ORB_SLAM::tests::setCamera(cameraMatrix(), imageWidth, imageHeight);
        const auto points = scenePoints();

        std::vector<cv::KeyPoint> firstKeyPoints;
        std::vector<cv::KeyPoint> secondKeyPoints;

        for (const cv::Point3f& point : points) {
            firstKeyPoints.emplace_back(project(point, 0), 1.0F);
            secondKeyPoints.emplace_back(project(point, secondCenter), 1.0F);
        }

        first = scene.addKeyFrame(poseAt(0), cameraMatrix(), firstKeyPoints,
                                  pyramid);
        second = scene.addKeyFrame(poseAt(secondCenter + offset),
                                   cameraMatrix(), secondKeyPoints, pyramid);

        for (std::size_t i = 0; i < points.size(); ++i) {
            scene.addMapPoint(points[i], {{first, i}, {second, i}});
        }

        first->UpdateConnections();
        second->UpdateConnections();
    }

    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* first;
    ORB_SLAM::KeyFrame* second;
};

// Returns how far the camera center of the keyframe moved from x on the
// x-axis.
float centerMovedFrom(ORB_SLAM::KeyFrame* keyFrame, float x)
{
    const cv::Mat center = keyFrame->GetCameraCenter();
    return static_cast<float>(cv::norm(center - cv::Mat{cv::Vec3f{x, 0, 0}}));
}

// A local bundle adjustment that is asked to stop before it starts leaves the
// poses as they are, while one that runs moves the perturbed pose.
void localBundleAdjustmentStopsOnRequest()
{
    constexpr float offset = 0.05F;
    constexpr float unchanged = 1e-5F;

    for (const bool stop : {false, true}) {
        PerturbedPair pair{offset};
        const std::atomic<bool> stopFlag{stop};
        const float start = 0.5F + offset;

        ORB_SLAM::Optimizer::LocalBundleAdjustment(pair.second, &stopFlag);

        const float moved = centerMovedFrom(pair.second, start);

        if (stop) {
            check(moved <= unchanged,
                  std::format("a stopped adjustment keeps the pose, moved {}",
                              moved));
        }
        else {
            check(moved > unchanged,
                  std::format("an adjustment moves the perturbed pose, moved {}",
                              moved));
        }
    }
}

// Another thread may ask a running local bundle adjustment to stop, which
// ThreadSanitizer checks for data races.
void localBundleAdjustmentStopsFromAnotherThread()
{
    constexpr int adjustments = 20;
    constexpr float offset = 0.05F;

    std::atomic<bool> stopFlag{false};
    std::atomic<bool> done{false};

    std::jthread stopping{[&stopFlag, &done] {
        while (!done) {
            stopFlag = !stopFlag;
            std::this_thread::yield();
        }
    }};

    for (int i = 0; i < adjustments; ++i) {
        PerturbedPair pair{offset};
        ORB_SLAM::Optimizer::LocalBundleAdjustment(pair.second, &stopFlag);
    }

    done = true;
}

// The similarity optimization weighs the observations of each keyframe by
// the inverse variance of their pyramid level. Observations of the second
// keyframe at a coarse level that are off by a few pixels lie within the
// threshold for that level and remain inliers.
void sim3KeepsCoarseInliers()
{
    constexpr float secondCenter = 0.5F;
    constexpr int coarseLevel = 4;
    // Two standard deviations at the coarse level, which has a standard
    // deviation of 1.2^4 = 2.07 pixels.
    constexpr float offset = 4;
    const std::array points = scenePoints();

    std::vector<cv::KeyPoint> firstKeyPoints;
    std::vector<cv::KeyPoint> secondKeyPoints;

    for (std::size_t i = 0; i < points.size(); ++i) {
        firstKeyPoints.emplace_back(project(points[i], 0), 1.0F, -1.0F, 0.0F, 0);

        cv::Point2f observed = project(points[i], secondCenter);
        observed.x += i % 2 == 0 ? offset : -offset;
        secondKeyPoints.emplace_back(observed, 1.0F, -1.0F, 0.0F, coarseLevel);
    }

    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const first =
        scene.addKeyFrame(poseAt(0), cameraMatrix(), firstKeyPoints, pyramid);
    ORB_SLAM::KeyFrame* const second = scene.addKeyFrame(
        poseAt(secondCenter), cameraMatrix(), secondKeyPoints, pyramid);

    std::vector<ORB_SLAM::MapPoint*> matches;

    for (std::size_t i = 0; i < points.size(); ++i) {
        matches.push_back(
            scene.addMapPoint(points[i], {{first, i}, {second, i}}));
    }

    // The second camera seen from the first one.
    g2o::Sim3 S12{Eigen::Matrix3d::Identity(),
                  Eigen::Vector3d{secondCenter, 0, 0}, 1};

    const int inliers = ORB_SLAM::Optimizer::OptimizeSim3(
        first, second, matches, S12, chiSquareThreshold);

    check(inliers == static_cast<int>(points.size()),
          std::format("all {} matches remain inliers, got {}", points.size(),
                      inliers));
}

} // namespace

int main()
{
    sim3KeepsCoarseInliers();
    localBundleAdjustmentStopsOnRequest();
    localBundleAdjustmentStopsFromAnotherThread();

    return ORB_SLAM::tests::exitStatus();
}
