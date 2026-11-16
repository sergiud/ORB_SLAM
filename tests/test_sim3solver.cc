// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests of the similarity RANSAC that loop closing runs.

#include <array>
#include <cmath>
#include <cstddef>
#include <format>
#include <numbers>
#include <vector>

#include <opencv2/core.hpp>

#include "Check.h"
#include "Frames.h"
#include "Scene.h"
#include "Sim3Solver.h"

namespace {

using ORB_SLAM::tests::check;

constexpr float focalLength = 500;
constexpr float principalPointX = 320;
constexpr float principalPointY = 240;

cv::Mat cameraMatrix()
{
    return cv::Mat_<float>({3, 3}, {focalLength, 0, principalPointX,
                                    0, focalLength, principalPointY,
                                    0, 0, 1});
}

// Exposes the inlier check of a similarity, so that a test checks it for a
// given similarity instead of one that RANSAC draws.
class CheckingSim3Solver : public ORB_SLAM::Sim3Solver
{
public:
    using ORB_SLAM::Sim3Solver::Sim3Solver;
    using ORB_SLAM::Sim3Solver::CheckInliers;
    using ORB_SLAM::Sim3Solver::computeT;
    using ORB_SLAM::Sim3Solver::mR12i;
    using ORB_SLAM::Sim3Solver::mT12i;
    using ORB_SLAM::Sim3Solver::mT21i;
    using ORB_SLAM::Sim3Solver::mvbInliersi;
};

// A match whose points project within the chi-square threshold of 9.21
// pixels squared of the finest level is an inlier, also where the error
// exceeds 9.
void keepsMatchesWithinThreshold()
{
    // Squared reprojection error between 9 and 9.21.
    constexpr double squaredError = 9.1;
    const std::array points{
        cv::Point3f{-1.0F, -0.5F, 4.0F}, cv::Point3f{-0.5F, 0.4F, 4.5F},
        cv::Point3f{0.0F, -0.3F, 5.0F}, cv::Point3f{0.5F, 0.6F, 4.2F}};
    // The last match is off by the squared error in both images.
    const std::size_t offset = points.size() - 1;

    std::vector<cv::KeyPoint> keyPoints(points.size(), cv::KeyPoint{0, 0, 1});

    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const first =
        scene.addKeyFrame(cv::Mat::eye(4, 4, CV_32F), cameraMatrix(), keyPoints);
    ORB_SLAM::KeyFrame* const second =
        scene.addKeyFrame(cv::Mat::eye(4, 4, CV_32F), cameraMatrix(), keyPoints);

    std::vector<ORB_SLAM::MapPoint*> matches;

    for (std::size_t i = 0; i < points.size(); ++i) {
        cv::Point3f moved = points[i];

        if (i == offset) {
            moved.x += static_cast<float>(std::sqrt(squaredError)) * moved.z /
                       focalLength;
        }

        scene.addMapPoint(points[i], {{first, i}});
        matches.push_back(scene.addMapPoint(moved, {{second, i}}));
    }

    // Both keyframes share the pose, so the identity relates them.
    CheckingSim3Solver solver{first, second, matches};
    solver.mT12i = cv::Mat::eye(4, 4, CV_32F);
    solver.mT21i = cv::Mat::eye(4, 4, CV_32F);
    solver.CheckInliers();

    check(solver.mvbInliersi.size() == points.size() &&
              solver.mvbInliersi[offset],
          std::format("a match off by {} pixels squared is an inlier",
                      squaredError));
}

// Points that coincide in both keyframes are related by the identity, whose
// rotation has no axis.
void identityFromCoincidingPoints()
{
    constexpr double tolerance = 1e-5;

    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const first =
        scene.addKeyFrame(cv::Mat::eye(4, 4, CV_32F), cameraMatrix());
    ORB_SLAM::KeyFrame* const second =
        scene.addKeyFrame(cv::Mat::eye(4, 4, CV_32F), cameraMatrix());
    CheckingSim3Solver solver{first, second, {}};

    // One point in each column.
    cv::Mat points = cv::Mat_<float>({3, 3}, {-1, 0.5F, 1,
                                              0.5F, -0.5F, 1,
                                              4, 4.5F, 5});
    cv::Mat same = points.clone();
    solver.computeT(points, same);

    const cv::Mat rotation = solver.mR12i;
    const bool finite = cv::checkRange(rotation);
    const double deviation =
        finite ? cv::norm(rotation, cv::Mat::eye(3, 3, rotation.type())) : 0;

    check(finite && deviation <= tolerance,
          std::format("coinciding points have the identity rotation, got a "
                      "{} rotation {} off",
                      finite ? "finite" : "non-finite", deviation));
}

// The rotation maps the points of the second keyframe onto those of the
// first.
void rotationMapsSecondOntoFirst()
{
    constexpr double tolerance = 1e-4;
    constexpr double angle = std::numbers::pi / 6;

    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* const first =
        scene.addKeyFrame(cv::Mat::eye(4, 4, CV_32F), cameraMatrix());
    ORB_SLAM::KeyFrame* const second =
        scene.addKeyFrame(cv::Mat::eye(4, 4, CV_32F), cameraMatrix());
    CheckingSim3Solver solver{first, second, {}};

    // Rotations about two axes compose into one about an axis off every
    // coordinate axis, so that every term of the rotation matters.
    const auto c = static_cast<float>(std::cos(angle));
    const auto s = static_cast<float>(std::sin(angle));
    const cv::Mat aboutZ = cv::Mat_<float>({3, 3}, {c, -s, 0,
                                                    s, c, 0,
                                                    0, 0, 1});
    const cv::Mat aboutX = cv::Mat_<float>({3, 3}, {1, 0, 0,
                                                    0, c, -s,
                                                    0, s, c});
    const cv::Mat rotation = aboutX * aboutZ;
    const cv::Mat secondPoints = cv::Mat_<float>({3, 3}, {-1, 0.5F, 1,
                                                          0.5F, -0.5F, 1,
                                                          4, 4.5F, 5});
    cv::Mat firstPoints = rotation * secondPoints;
    cv::Mat secondCopy = secondPoints.clone();
    solver.computeT(firstPoints, secondCopy);

    const double deviation = cv::norm(solver.mR12i, rotation);

    check(deviation <= tolerance,
          std::format("the rotation maps the second points onto the first, "
                      "got one {} off",
                      deviation));
}

} // namespace

int main()
{
    identityFromCoincidingPoints();
    rotationMapsSecondOntoFirst();
    keepsMatchesWithinThreshold();

    return ORB_SLAM::tests::exitStatus();
}
