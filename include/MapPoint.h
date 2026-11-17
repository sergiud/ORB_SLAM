// SPDX-FileCopyrightText: 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
// SPDX-FileCopyrightText: 2015-2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include<opencv2/core.hpp>
#include"KeyFrame.h"
#include"Map.h"
#include "ORB_SLAM_export.h"

#include <mutex>
#include <unordered_set>
#include <vector>


namespace ORB_SLAM
{

class ImageFeature;
class KeyFrame;
class Map;


class ORB_SLAM_EXPORT MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    /**
     * Removes the observations by the keyframes of a batch that the map
     * frees. A reference keyframe among them gives way to another observer.
     */
    void RemoveReferences(const std::unordered_set<KeyFrame*>& keyFrames);

    void Replace(MapPoint* pMP);

    void IncreaseVisible();
    void IncreaseFound();
    float GetFoundRatio();

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    /**
     * Predicts the pyramid level at which a camera at distance @p dist
     * detects this point.
     *
     * @param dist The distance between the camera center and the point.
     * @param scaleFactors The scale factor of each pyramid level of the camera.
     */
    int PredictScale(float dist, const std::vector<float>& scaleFactors);

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    bool mbTrackInView{false};
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF{0};

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
