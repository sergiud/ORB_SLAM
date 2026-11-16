// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests of the descriptor matching between frames and keyframes.

#include <array>
#include <format>
#include <set>
#include <string_view>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "Check.h"
#include "Frames.h"
#include "ORBmatcher.h"
#include "Scene.h"

namespace {

using ORB_SLAM::tests::check;

constexpr int fullTurn = 360;
// Width of a bin of the rotation histogram in degrees.
constexpr float binWidth = 12;

// Exposes how the matcher sorts rotations into its histogram.
class RotationMatcher : public ORB_SLAM::ORBmatcher
{
public:
    using ORB_SLAM::ORBmatcher::HISTO_LENGTH;
    using ORB_SLAM::ORBmatcher::RotationBin;
};

// The rotations of a full turn fill every bin of the histogram.
void rotationsReachEveryBin()
{
    std::set<int> bins;

    for (int angle = 0; angle < fullTurn; ++angle) {
        bins.insert(RotationMatcher::RotationBin(static_cast<float>(angle), 0));
    }

    check(static_cast<int>(bins.size()) == RotationMatcher::HISTO_LENGTH,
          std::format("a full turn reaches {} bins, got {}",
                      RotationMatcher::HISTO_LENGTH, bins.size()));
}

// Small rotations in either direction share the bin of no rotation, so that
// a camera that barely rolls has one peak in the histogram.
void smallRotationsShareBin()
{
    constexpr float small = 5;

    const int clockwise = RotationMatcher::RotationBin(0, small);
    const int counterclockwise = RotationMatcher::RotationBin(small, 0);

    check(clockwise == 0 && counterclockwise == 0,
          std::format("rotations of -{0} and {0} degrees fall into bin 0, got "
                      "{1} and {2}",
                      small, clockwise, counterclockwise));
}

// Rotations one bin width apart fall into neighboring bins.
void rotationsByBinWidthAreNeighbors()
{
    const int counterclockwise = RotationMatcher::RotationBin(binWidth, 0);
    const int clockwise = RotationMatcher::RotationBin(0, binWidth);
    const int last = RotationMatcher::HISTO_LENGTH - 1;

    check(counterclockwise == 1,
          std::format("a rotation of {} degrees falls into bin 1, got {}",
                      binWidth, counterclockwise));
    check(clockwise == last,
          std::format("a rotation of -{} degrees falls into bin {}, got {}",
                      binWidth, last, clockwise));
}

constexpr int width = 640;
constexpr int height = 480;
constexpr float focalLength = 500;
constexpr ORB_SLAM::tests::Pyramid pyramid{1.2F, 8};
// Search radius in pixels at the finest level.
constexpr float radius = 15;
// Largest descriptor distance a match may have.
constexpr int maxDistance = 100;
// Distance of the point from the current camera and from the camera that
// observed it.
constexpr float depth = 2;

cv::Mat cameraMatrix()
{
    return cv::Mat_<float>({3, 3}, {focalLength, 0, width / 2.0F,
                                    0, focalLength, height / 2.0F,
                                    0, 0, 1});
}

// Returns the pose of a camera centered at z on the z-axis, looking along z.
cv::Mat poseAt(float z)
{
    cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
    Tcw.at<float>(2, 3) = -z;
    return Tcw;
}

cv::Point2f project(const cv::Point3f& point, float cameraZ)
{
    const float z = point.z - cameraZ;
    return {focalLength * point.x / z + width / 2.0F,
            focalLength * point.y / z + height / 2.0F};
}

// Where a map point lies relative to the current camera.
enum class Side { front, behind, center };

// A map point at a distance from the current camera, which an earlier camera
// behind it observed from the same distance at the same pyramid level, and a
// keypoint of the current frame where the point appears when projected
// through the camera center.
struct ProjectedPoint
{
    explicit ProjectedPoint(Side side)
        : position{positionAt(side)}
    {
        ORB_SLAM::tests::setCamera(cameraMatrix(), width, height);

        const float observerZ = position.z - depth;
        observer = scene.addKeyFrame(
            poseAt(observerZ), cameraMatrix(),
            {cv::KeyPoint{project(position, observerZ), 1, -1, 0, level}},
            pyramid);
        point = scene.addMapPoint(position, {{observer, 0}});
        point->ComputeDistinctiveDescriptors();
        point->UpdateNormalAndDepth();

        // A point at the camera center has no projection, so the keypoint
        // lies at the principal point.
        const cv::Point2f keyPoint = side == Side::center
            ? cv::Point2f{width / 2.0F, height / 2.0F}
            : project(position, 0);
        current = ORB_SLAM::tests::makeFrame(
            poseAt(0), cameraMatrix(), {cv::KeyPoint{keyPoint, 1, -1, 0, level}},
            pyramid);
    }

    static cv::Point3f positionAt(Side side)
    {
        switch (side) {
        case Side::front:
            return {0.5F, 0.3F, depth};
        case Side::behind:
            return {0.5F, 0.3F, -depth};
        case Side::center:
            return {0, 0, 0};
        }

        std::unreachable();
    }

    // Returns the last frame, which observed the point like the keyframe.
    ORB_SLAM::Frame lastFrame() const
    {
        ORB_SLAM::Frame last = ORB_SLAM::tests::makeFrame(
            observer->GetPose(), cameraMatrix(), observer->GetKeyPointsUn(),
            pyramid);
        last.mvpMapPoints = {point};
        return last;
    }

    static constexpr int level = 1;
    cv::Point3f position;
    ORB_SLAM::tests::Scene scene;
    ORB_SLAM::KeyFrame* observer;
    ORB_SLAM::MapPoint* point;
    ORB_SLAM::Frame current;
};

// A search matches a point in front of the camera and ignores one behind it,
// even where both project onto the same keypoint.
void checkMatches(Side side, int matches, std::string_view search)
{
    const int expected = side == Side::front ? 1 : 0;
    constexpr std::array<std::string_view, 3> where{
        "in front of the camera", "behind the camera", "at the camera center"};

    check(matches == expected,
          std::format("{} matches {} point {}, got {}", search, expected,
                      where[std::to_underlying(side)], matches));
}

// Tracking with the motion model searches the points of the last frame.
void lastFrameSearch(Side side)
{
    ProjectedPoint setup{side};
    const ORB_SLAM::Frame last = setup.lastFrame();
    ORB_SLAM::ORBmatcher matcher{0.9F, /*checkOri=*/false};

    checkMatches(side, matcher.SearchByProjection(setup.current, last, radius),
                 "the search in the last frame");
}

// Tracking the previous frame searches its points in a window.
void windowSearch(Side side)
{
    ProjectedPoint setup{side};
    ORB_SLAM::Frame last = setup.lastFrame();
    ORB_SLAM::ORBmatcher matcher{0.9F, /*checkOri=*/false};
    std::vector<ORB_SLAM::MapPoint*> matches;

    checkMatches(side,
                 matcher.SearchByProjection(last, setup.current,
                                            static_cast<int>(radius), matches),
                 "the search in a window");
}

// Relocalisation searches the points of a candidate keyframe.
void keyFrameSearch(Side side)
{
    ProjectedPoint setup{side};
    ORB_SLAM::ORBmatcher matcher{0.9F, /*checkOri=*/false};

    checkMatches(side,
                 matcher.SearchByProjection(setup.current, setup.observer, {},
                                            radius, maxDistance),
                 "the search in a keyframe");
}

// Returns an ORB descriptor with the first bits set, which is that many bits
// away from one of zeros.
cv::Mat descriptorWithBits(int bits)
{
    constexpr int bitsPerByte = 8;

    cv::Mat descriptor = cv::Mat::zeros(1, ORB_SLAM::tests::descriptorBytes, CV_8U);
    descriptor.colRange(0, bits / bitsPerByte).setTo(0xFF);
    return descriptor;
}

// The rotation check of the initialization counts the matches that remain,
// not those that a better match replaced. Three replaced matches would
// otherwise outweigh a rotation that two correct matches each share.
void initializationCountsRemainingMatches()
{
    constexpr int window = 10;
    // Keypoints that compete for the same keypoint, each closer to it than
    // the one before, so that each replaces the match of the one before.
    constexpr std::array competingDistances{40, 30, 20, 10};
    // Rotations of 0, 60, 120 and 180 degrees fall into distinct bins.
    constexpr std::array rotations{0.0F, 60.0F, 120.0F, 180.0F};
    constexpr int pairsPerRotation = 2;
    constexpr float spacing = 100;

    ORB_SLAM::tests::setCamera(cameraMatrix(), width, height);

    std::vector<cv::KeyPoint> first;
    std::vector<cv::KeyPoint> second;
    std::vector<cv::Point2f> previous;
    cv::Mat firstDescriptors;
    cv::Mat secondDescriptors;

    const cv::Point2f contested{spacing, spacing};
    second.emplace_back(contested, 1.0F, 0.0F);
    secondDescriptors.push_back(descriptorWithBits(0));

    for (const int distance : competingDistances) {
        first.emplace_back(contested, 1.0F, rotations[0]);
        previous.push_back(contested);
        firstDescriptors.push_back(descriptorWithBits(distance));
    }

    for (std::size_t r = 1; r < rotations.size(); ++r) {
        for (int p = 0; p < pairsPerRotation; ++p) {
            const cv::Point2f at{spacing * (2 + p), spacing * r};
            first.emplace_back(at, 1.0F, rotations[r]);
            second.emplace_back(at, 1.0F, 0.0F);
            previous.push_back(at);
            firstDescriptors.push_back(descriptorWithBits(0));
            secondDescriptors.push_back(descriptorWithBits(0));
        }
    }

    ORB_SLAM::Frame firstFrame = ORB_SLAM::tests::makeFrame(
        cv::Mat::eye(4, 4, CV_32F), cameraMatrix(), first, pyramid,
        firstDescriptors);
    ORB_SLAM::Frame secondFrame = ORB_SLAM::tests::makeFrame(
        cv::Mat::eye(4, 4, CV_32F), cameraMatrix(), second, pyramid,
        secondDescriptors);

    ORB_SLAM::ORBmatcher matcher{0.9F, /*checkOri=*/true};
    std::vector<int> matches;
    const int matched = matcher.SearchForInitialization(
        firstFrame, secondFrame, previous, matches, window);

    const int expected = (rotations.size() - 1) * pairsPerRotation;

    check(matched == expected,
          std::format("the {} matches of the three common rotations remain, "
                      "got {}",
                      expected, matched));
}

} // namespace

int main()
{
    rotationsReachEveryBin();
    smallRotationsShareBin();
    rotationsByBinWidthAreNeighbors();
    initializationCountsRemainingMatches();

    for (const Side side : {Side::front, Side::behind, Side::center}) {
        lastFrameSearch(side);
        windowSearch(side);
        keyFrameSearch(side);
    }

    return ORB_SLAM::tests::exitStatus();
}
