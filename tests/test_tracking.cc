// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests of the tracking front end that need no images.

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <filesystem>
#include <format>
#include <fstream>
#include <memory>
#include <new>
#include <optional>
#include <print>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "Check.h"
#include "FramePublisher.h"
#include "KeyFrameDatabase.h"
#include "LogCapture.h"
#include "Frames.h"
#include "LocalMapping.h"
#include "Map.h"
#include "MapPoint.h"
#include "Scene.h"
#include "Storage.h"
#include "Tracking.h"

namespace {

using ORB_SLAM::tests::check;

constexpr double framesPerSecond = 30;
constexpr int features = 1000;
constexpr float scaleFactor = 1.2F;
constexpr int levels = 8;
constexpr int fastThreshold = 20;
constexpr int harrisScore = 1;

// Creates a tracker for the camera matrix without vocabulary, publishers
// and map, which construction does not use.
ORB_SLAM::Tracking makeTracking(const cv::Mat& K, bool useMotionModel)
{
    return ORB_SLAM::Tracking{nullptr, nullptr, nullptr, K,
                              cv::Mat{}, framesPerSecond, features,
                              scaleFactor, levels, fastThreshold,
                              harrisScore, useMotionModel};
}

// Construction logs whether the motion model is used, and warns if not.
void logsMotionModel(bool useMotionModel, nglog::LogSeverity severity,
                     std::string_view expected)
{
    ORB_SLAM::tests::LogCapture capture;
    makeTracking(cv::Mat1f::eye(3, 3), useMotionModel);

    check(capture.contains(severity, expected),
          std::format("the log has {:?} at severity {}", expected,
                      nglog::experimental::Name(nglog::experimental::Severity{severity})));
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

// Distortion coefficients other than the 4 or 5 of OpenCV's model are
// rejected with their number, while those are accepted.
void rejectsDistortion(const cv::Mat& distortion, std::string_view expected)
{
    std::string message;

    try {
        ORB_SLAM::Tracking{nullptr, nullptr, nullptr,
                           cv::Mat1f::eye(3, 3), distortion, framesPerSecond,
                           features, scaleFactor, levels, fastThreshold,
                           harrisScore, /*useMotionModel=*/true};
    }
    catch (const std::invalid_argument& e) {
        message = e.what();
    }

    check(message == expected,
          std::format("the error reads {:?}, got {:?}", expected, message));
}

// Exposes the distortion the tracker undistorts keypoints with.
class DistortionTracking : public ORB_SLAM::Tracking
{
public:
    using ORB_SLAM::Tracking::Tracking;
    using ORB_SLAM::Tracking::mDistCoef;
};

// A camera without distortion coefficients has no distortion, as a
// calibration without them describes.
void emptyDistortionMeansNone()
{
    constexpr int coefficients = 4;

    std::optional<DistortionTracking> tracking;

    try {
        tracking.emplace(nullptr, nullptr, nullptr,
                         cv::Mat1f::eye(3, 3), cv::Mat{}, framesPerSecond,
                         features, scaleFactor, levels, fastThreshold,
                         harrisScore, /*useMotionModel=*/true);
    }
    catch (const std::invalid_argument& e) {
        check(false, std::format("a camera without distortion coefficients is "
                                 "accepted, got {:?}",
                                 e.what()));
        return;
    }

    const cv::Mat& distortion = tracking->mDistCoef;

    check(distortion.total() == coefficients && distortion.type() == CV_32F &&
              cv::countNonZero(distortion) == 0,
          std::format("the camera has {} zero distortion coefficients, got {} "
                      "of type {}",
                      coefficients, distortion.total(), distortion.type()));
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

// Exposes how the tracker hands keyframes to local mapping.
class KeyFrameTracking : public ORB_SLAM::Tracking
{
public:
    using ORB_SLAM::Tracking::Tracking;
    using ORB_SLAM::Tracking::CreateNewKeyFrame;
    using ORB_SLAM::Tracking::mpLastKeyFrame;
    using ORB_SLAM::Tracking::mnLastKeyFrameId;
};

// A keyframe that local mapping refuses while loop closing pauses it is not
// the last keyframe of the tracker, which would otherwise refer to a
// keyframe nobody processes or owns.
void dropsRefusedKeyFrame()
{
    constexpr unsigned lastKeyFrameId = 7;
    constexpr long unsigned frameId = 9;

    ORB_SLAM::Map map;
    ORB_SLAM::LocalMapping localMapping{&map};
    KeyFrameTracking tracking{nullptr, nullptr, &map,
                              cv::Mat1f::eye(3, 3), cv::Mat{},
                              framesPerSecond, features, scaleFactor, levels,
                              fastThreshold, harrisScore,
                              /*useMotionModel=*/true};

    tracking.SetLocalMapper(&localMapping);
    tracking.mnLastKeyFrameId = lastKeyFrameId;

    tracking.mCurrentFrame =
        ORB_SLAM::tests::makeFrame(cv::Mat::eye(4, 4, CV_32F));
    tracking.mCurrentFrame.mnId = frameId;

    localMapping.RequestPause();
    tracking.CreateNewKeyFrame();

    check(tracking.mpLastKeyFrame == nullptr,
          "a refused keyframe does not become the last keyframe");
    check(tracking.mnLastKeyFrameId == lastKeyFrameId,
          std::format("the last keyframe id stays {}, got {}", lastKeyFrameId,
                      tracking.mnLastKeyFrameId));
}

// Exposes the collaborators and the state a tracker starts with.
class InitialTracking : public ORB_SLAM::Tracking
{
public:
    using ORB_SLAM::Tracking::Tracking;
    using ORB_SLAM::Tracking::mpInitializer;
    using ORB_SLAM::Tracking::mpKeyFrameDB;
    using ORB_SLAM::Tracking::mpLastKeyFrame;
    using ORB_SLAM::Tracking::mpLocalMapper;
    using ORB_SLAM::Tracking::mpLoopClosing;
    using ORB_SLAM::Tracking::mpReferenceKF;
};

// A tracker constructed from a settings file starts without collaborators
// and keyframes. The tracker is constructed in storage filled with a pattern
// other than zero, so that a pointer the constructor leaves uninitialized
// shows.
void settingsTrackerStartsEmpty()
{
    const std::filesystem::path settings =
        std::filesystem::temp_directory_path() / "orb_slam_test_settings.yml";
    {
        std::ofstream stream{settings};
        std::print(stream,
                   "%YAML:1.0\n"
                   "Camera.fx: 500.0\n"
                   "Camera.fy: 510.0\n"
                   "Camera.cx: 320.0\n"
                   "Camera.cy: 240.0\n"
                   "ORBextractor.nFeatures: {}\n"
                   "ORBextractor.scaleFactor: {}\n"
                   "ORBextractor.nLevels: {}\n"
                   "ORBextractor.fastTh: {}\n"
                   "ORBextractor.nScoreType: {}\n",
                   features, scaleFactor, levels, fastThreshold, harrisScore);
    }

    alignas(InitialTracking) std::byte storage[sizeof(InitialTracking)];
    ORB_SLAM::tests::fillWithPattern(storage);

    ORB_SLAM::tests::LogCapture capture;
    InitialTracking* const tracking = new (storage)
        InitialTracking{nullptr, nullptr, nullptr, settings.string()};

    check(capture.contains(NGLOG_INFO, "camera parameters: fx 500, fy 510"),
          "the tracker logs the camera parameters of its settings");
    check(capture.contains(NGLOG_INFO, std::format("ORB extractor parameters: "
                                                   "{} features",
                                                   features)),
          "the tracker logs the extractor parameters of its settings");
    check(capture.contains(NGLOG_WARNING, "motion model: disabled"),
          "the tracker warns that its settings disable the motion model");

    check(tracking->mpInitializer == nullptr,
          "the tracker starts without initializer");
    check(tracking->mpLocalMapper == nullptr,
          "the tracker starts without local mapping");
    check(tracking->mpLoopClosing == nullptr,
          "the tracker starts without loop closing");
    check(tracking->mpKeyFrameDB == nullptr,
          "the tracker starts without keyframe database");
    check(tracking->mpLastKeyFrame == nullptr,
          "the tracker starts without last keyframe");
    check(tracking->mpReferenceKF == nullptr,
          "the tracker starts without reference keyframe");

    std::destroy_at(tracking);
    std::filesystem::remove(settings);
}

// The frames the tracker makes from images carry the time of the image.
void framesCarryImageTime()
{
    constexpr double timestamp = 1.5;
    constexpr int width = 640;
    constexpr int height = 480;

    ORB_SLAM::Map map;
    ORB_SLAM::FramePublisher framePublisher;
    ORB_SLAM::Tracking tracking{nullptr, &framePublisher, &map,
                                cv::Mat1f::eye(3, 3), cv::Mat{},
                                framesPerSecond, features, scaleFactor, levels,
                                fastThreshold, harrisScore,
                                /*useMotionModel=*/true};

    tracking.Track(cv::Mat::zeros(height, width, CV_8U), timestamp);

    check(tracking.mCurrentFrame.mTimeStamp == timestamp,
          std::format("the frame carries the time {} of its image, got {}",
                      timestamp, tracking.mCurrentFrame.mTimeStamp));
}

// Loop closing forces a relocalisation while the tracker processes images,
// which ThreadSanitizer checks for data races.
void relocalisationForcedConcurrently()
{
    constexpr int images = 50;
    constexpr int width = 640;
    constexpr int height = 480;

    ORB_SLAM::Map map;
    ORB_SLAM::FramePublisher framePublisher;
    ORB_SLAM::Tracking tracking{nullptr, &framePublisher, &map,
                                cv::Mat1f::eye(3, 3), cv::Mat{},
                                framesPerSecond, features, scaleFactor, levels,
                                fastThreshold, harrisScore,
                                /*useMotionModel=*/true};
    const cv::Mat black = cv::Mat::zeros(height, width, CV_8U);

    std::atomic<bool> done{false};

    std::jthread loopClosing{[&tracking, &done] {
        while (!done) {
            tracking.ForceRelocalisation();
        }
    }};

    for (int i = 0; i < images; ++i) {
        tracking.Track(black, i);
    }

    done = true;
}

// Exposes the pointers to keyframes and map points that the tracker keeps
// from one image to the next.
class ReferringTracking : public ORB_SLAM::Tracking
{
public:
    using ORB_SLAM::Tracking::Tracking;
    using ORB_SLAM::Tracking::DropErasedObjects;
    using ORB_SLAM::Tracking::Relocalisation;
    using ORB_SLAM::Tracking::Reset;
    using ORB_SLAM::Tracking::mInitialFrame;
    using ORB_SLAM::Tracking::mLastFrame;
    using ORB_SLAM::Tracking::mpLastKeyFrame;
    using ORB_SLAM::Tracking::mpReferenceKF;
    using ORB_SLAM::Tracking::mvpLocalKeyFrames;
    using ORB_SLAM::Tracking::mvpLocalMapPoints;
};

// After an image, the tracker keeps no erased keyframe or map point, which
// the map frees once the tracker passed two images. Until the tracker
// passes an image, the map keeps the references to them.
void trackerDropsErasedObjects()
{
    constexpr int width = 640;
    constexpr int height = 480;

    ORB_SLAM::tests::Scene scene;
    const std::vector<cv::KeyPoint> keyPoint{cv::KeyPoint{0, 0, 1}};
    ORB_SLAM::KeyFrame* const root = scene.addKeyFrame(
        cv::Mat::eye(4, 4, CV_32F), cv::Mat::eye(3, 3, CV_32F), keyPoint);
    ORB_SLAM::KeyFrame* const erased = scene.addKeyFrame(
        cv::Mat::eye(4, 4, CV_32F), cv::Mat::eye(3, 3, CV_32F), keyPoint);
    erased->ChangeParent(root);

    ORB_SLAM::FramePublisher framePublisher;
    ReferringTracking tracking{nullptr, &framePublisher, &scene.map(),
                               cv::Mat1f::eye(3, 3), cv::Mat{},
                               framesPerSecond, features, scaleFactor, levels,
                               fastThreshold, harrisScore,
                               /*useMotionModel=*/true};

    // The map owns the point, and the root keeps a reference to it that the
    // erasure misses.
    auto* const point = new ORB_SLAM::MapPoint{cv::Mat{cv::Vec3f{0, 0, 1}},
                                               root, &scene.map()};
    scene.map().AddMapPoint(point);
    root->AddMapPoint(point, 0);

    tracking.mLastFrame = ORB_SLAM::tests::makeFrame(
        cv::Mat::eye(4, 4, CV_32F), cv::Mat{}, keyPoint);
    tracking.mLastFrame.mvpMapPoints = {point};
    tracking.mInitialFrame = tracking.mLastFrame;
    tracking.mpLastKeyFrame = erased;
    tracking.mpReferenceKF = erased;
    tracking.mvpLocalKeyFrames = {root, erased};
    tracking.mvpLocalMapPoints = {point};

    point->SetBadFlag();
    erased->SetBadFlag();

    for (int i = 0; i < 2; ++i) {
        scene.map().Reclaim();
    }

    check(root->GetMapPoint(0) == point,
          "the map keeps the references to an erased map point while the "
          "tracker passed no image");

    const cv::Mat black = cv::Mat::zeros(height, width, CV_8U);
    tracking.Track(black, 0);

    check(tracking.mLastFrame.mvpMapPoints[0] == nullptr,
          "the last frame keeps no erased map point");
    check(tracking.mInitialFrame.mvpMapPoints[0] == nullptr,
          "the initial frame keeps no erased map point");
    check(tracking.mpLastKeyFrame == nullptr,
          "the tracker keeps no erased last keyframe");
    check(tracking.mpReferenceKF == nullptr,
          "the tracker keeps no erased reference keyframe");
    check(tracking.mvpLocalKeyFrames == std::vector<ORB_SLAM::KeyFrame*>{root},
          "the local map keeps no erased keyframe");
    check(tracking.mvpLocalMapPoints.empty(),
          "the local map keeps no erased map point");

    scene.map().Reclaim();

    check(root->GetMapPoint(0) == nullptr,
          "the map removes the references to an erased map point once the "
          "tracker passed an image");
}

// The current frame of the tracker keeps no erased map point after its
// quiescent point either.
void currentFrameDropsErasedMapPoints()
{
    ORB_SLAM::tests::Scene scene;
    const std::vector<cv::KeyPoint> keyPoint{cv::KeyPoint{0, 0, 1}};
    ORB_SLAM::KeyFrame* const keyFrame = scene.addKeyFrame(
        cv::Mat::eye(4, 4, CV_32F), cv::Mat::eye(3, 3, CV_32F), keyPoint);
    ORB_SLAM::MapPoint* const point =
        scene.addMapPoint({0, 0, 1}, {{keyFrame, 0}});

    ReferringTracking tracking{nullptr, nullptr, &scene.map(),
                               cv::Mat1f::eye(3, 3), cv::Mat{},
                               framesPerSecond, features, scaleFactor, levels,
                               fastThreshold, harrisScore,
                               /*useMotionModel=*/true};
    tracking.mCurrentFrame = ORB_SLAM::tests::makeFrame(
        cv::Mat::eye(4, 4, CV_32F), cv::Mat{}, keyPoint);
    tracking.mCurrentFrame.mvpMapPoints = {point};

    point->SetBadFlag();
    tracking.DropErasedObjects();

    check(tracking.mCurrentFrame.mvpMapPoints[0] == nullptr,
          "the current frame keeps no erased map point");
}

// A reset destroys every keyframe and map point, so the tracker keeps none
// across it.
void trackerForgetsMapOnReset()
{
    ORB_SLAM::Map map;
    ORB_SLAM::ORBVocabulary vocabulary;
    ORB_SLAM::KeyFrameDatabase database{vocabulary};
    ReferringTracking tracking{nullptr, nullptr, &map,
                               cv::Mat1f::eye(3, 3), cv::Mat{},
                               framesPerSecond, features, scaleFactor, levels,
                               fastThreshold, harrisScore,
                               /*useMotionModel=*/true};
    tracking.SetKeyFrameDatabase(&database);

    const std::vector<cv::KeyPoint> keyPoint{cv::KeyPoint{0, 0, 1}};
    ORB_SLAM::Frame frame = ORB_SLAM::tests::makeFrame(
        cv::Mat::eye(4, 4, CV_32F), cv::Mat{}, keyPoint);
    auto* const keyFrame = new ORB_SLAM::KeyFrame{frame, &map, &database};
    map.AddKeyFrame(keyFrame);
    auto* const point =
        new ORB_SLAM::MapPoint{cv::Mat{cv::Vec3f{0, 0, 1}}, keyFrame, &map};
    map.AddMapPoint(point);

    frame.mvpMapPoints = {point};
    tracking.mCurrentFrame = frame;
    tracking.mLastFrame = frame;
    tracking.mInitialFrame = frame;
    tracking.mpReferenceKF = keyFrame;
    tracking.mpLastKeyFrame = keyFrame;
    tracking.mvpLocalKeyFrames = {keyFrame};
    tracking.mvpLocalMapPoints = {point};

    tracking.Reset();

    const auto none = [](const ORB_SLAM::Frame& tracked) {
        return std::ranges::all_of(tracked.mvpMapPoints,
                                   [](const ORB_SLAM::MapPoint* mapPoint) {
                                       return mapPoint == nullptr;
                                   });
    };

    check(none(tracking.mCurrentFrame) && none(tracking.mLastFrame) &&
              none(tracking.mInitialFrame),
          "the frames of the tracker keep no map point after a reset");
    check(tracking.mpReferenceKF == nullptr && tracking.mpLastKeyFrame == nullptr,
          "the tracker keeps no reference or last keyframe after a reset");
    check(tracking.mvpLocalKeyFrames.empty() && tracking.mvpLocalMapPoints.empty(),
          "the local map is empty after a reset");
}

// A forced relocalisation without a last keyframe, which an erasure dropped,
// finds no candidate rather than reading the missing keyframe.
void forcedRelocalisationWithoutLastKeyFrame()
{
    constexpr int branching = 2;
    constexpr int depth = 1;

    ORB_SLAM::ORBVocabulary vocabulary;
    vocabulary.create(
        {{cv::Mat::zeros(1, ORB_SLAM::tests::descriptorBytes, CV_8U),
          cv::Mat::ones(1, ORB_SLAM::tests::descriptorBytes, CV_8U)}},
        branching, depth);
    ORB_SLAM::Map map;
    ORB_SLAM::KeyFrameDatabase database{vocabulary};
    ReferringTracking tracking{&vocabulary, nullptr, &map,
                               cv::Mat1f::eye(3, 3), cv::Mat{},
                               framesPerSecond, features, scaleFactor, levels,
                               fastThreshold, harrisScore,
                               /*useMotionModel=*/true};
    tracking.SetKeyFrameDatabase(&database);
    tracking.mCurrentFrame = ORB_SLAM::tests::makeFrame(
        cv::Mat::eye(4, 4, CV_32F), cv::Mat{}, {cv::KeyPoint{0, 0, 1}});
    tracking.mCurrentFrame.mpORBvocabulary = &vocabulary;
    tracking.mpLastKeyFrame = nullptr;

    tracking.ForceRelocalisation();

    check(!tracking.Relocalisation(),
          "a forced relocalisation without a last keyframe fails");
}

} // namespace

int main()
{
    // The other tests construct trackers without distortion coefficients.
    emptyDistortionMeansNone();
    logsMotionModel(true, NGLOG_INFO, "motion model: enabled");
    logsMotionModel(false, NGLOG_WARNING,
                    "motion model: disabled (not recommended)");
    rejectsCameraMatrix(cv::Mat1f::eye(2, 3),
                        "camera intrinsic matrix must be 3x3, got 2x3");
    rejectsCameraMatrix(cv::Mat{}, "camera intrinsic matrix must be 3x3, got 0x0");
    rejectsDistortion(cv::Mat1f::zeros(1, 3),
                      "distortion coefficients must hold 4 or 5 values, got 3");
    rejectsDistortion(cv::Mat1f::zeros(1, 8),
                      "distortion coefficients must hold 4 or 5 values, got 8");
    rejectsDistortion(cv::Mat1f::zeros(1, 4), "");
    rejectsDistortion(cv::Mat1f::zeros(1, 5), "");
    keyPointsOfInliers();
    dropsRefusedKeyFrame();
    settingsTrackerStartsEmpty();
    framesCarryImageTime();
    relocalisationForcedConcurrently();
    trackerDropsErasedObjects();
    currentFrameDropsErasedMapPoints();
    trackerForgetsMapOnReset();
    forcedRelocalisationWithoutLastKeyFrame();

    return ORB_SLAM::tests::exitStatus();
}
