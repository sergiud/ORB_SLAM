// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef ORB_SLAM_TESTS_SCENE_H
#define ORB_SLAM_TESTS_SCENE_H

#include <cstddef>
#include <initializer_list>
#include <memory>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "Frames.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "MapPoint.h"
#include "ORBVocabulary.h"

namespace ORB_SLAM::tests {

//! Keypoint observation of a map point by a keyframe.
struct Observation
{
    KeyFrame* keyFrame;
    std::size_t index;
};

/**
 * Owns the keyframes and map points of a test, which the library only
 * refers to.
 */
class Scene
{
public:
    Scene() = default;

    //! Creates a scene whose database indexes words of @p vocabulary.
    explicit Scene(ORBVocabulary vocabulary)
        : vocabulary_{std::move(vocabulary)}
    {
    }

    Scene(const Scene&) = delete;
    Scene& operator=(const Scene&) = delete;

    /**
     * Adds a keyframe made from undistorted keypoints, as @ref makeFrame
     * describes.
     */
    KeyFrame* addKeyFrame(const cv::Mat& Tcw, const cv::Mat& K,
                          std::vector<cv::KeyPoint> keyPoints = {},
                          Pyramid pyramid = {})
    {
        Frame frame = makeFrame(Tcw, K, std::move(keyPoints), pyramid);
        keyFrames_.push_back(std::make_unique<KeyFrame>(frame, &map_, &database_));
        map_.AddKeyFrame(keyFrames_.back().get());
        return keyFrames_.back().get();
    }

    /**
     * Adds a map point at @p position that the keyframes observe. The first
     * observing keyframe is its reference.
     */
    MapPoint* addMapPoint(const cv::Point3f& position,
                          std::initializer_list<Observation> observations)
    {
        mapPoints_.push_back(std::make_unique<MapPoint>(
            cv::Mat{cv::Vec3f{position.x, position.y, position.z}},
            observations.begin()->keyFrame, &map_));
        MapPoint* const mapPoint = mapPoints_.back().get();

        for (const Observation& observation : observations) {
            observation.keyFrame->AddMapPoint(mapPoint, observation.index);
            mapPoint->AddObservation(observation.keyFrame, observation.index);
        }

        return mapPoint;
    }

    //! Returns the map that holds the keyframes.
    Map& map()
    {
        return map_;
    }

    //! Returns the database that indexes the keyframes by their words.
    KeyFrameDatabase& database()
    {
        return database_;
    }

private:
    Map map_;
    ORBVocabulary vocabulary_;
    KeyFrameDatabase database_{vocabulary_};
    std::vector<std::unique_ptr<KeyFrame>> keyFrames_;
    std::vector<std::unique_ptr<MapPoint>> mapPoints_;
};

} // namespace ORB_SLAM::tests

#endif // ORB_SLAM_TESTS_SCENE_H
