// SPDX-FileCopyrightText: 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
// SPDX-FileCopyrightText: 2015-2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef FRAME_H
#define FRAME_H

#include <BowVector.h>
#include <FeatureVector.h>

#include "MapPoint.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "ORB_SLAM_export.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;
class KeyFrameDatabase;

class ORB_SLAM_EXPORT Frame
{
public:
    Frame();
    Frame(const Frame &frame);
    Frame(const cv::Mat &im, const double &timeStamp, ORBextractor* extractor, ORBVocabulary* voc, const cv::Mat &K, const cv::Mat &distCoef);

    ORBVocabulary* mpORBvocabulary;
    ORBextractor* mpORBextractor;

    // Frame image
    cv::Mat im;

    // Frame timestamp
    double mTimeStamp;

    // Calibration Matrix and k1,k2,p1,p2 Distortion Parameters
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    cv::Mat mDistCoef;

    // Number of KeyPoints
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system)
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;

    // Bag of Words Vector structures
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint
    cv::Mat mDescriptors;

    // MapPoints associated to keypoints, NULL pointer if not association
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera Pose
    cv::Mat mTcw;

    // Current and Next Frame id
    static long unsigned int nNextId;
    long unsigned int mnId;

    KeyFrame* mpReferenceKF;

    void ComputeBoW();

    void UpdatePoseMatrices();

    // Check if a MapPoint is in the frustum of the camera and also fills variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Scale Pyramid Info
    int mnScaleLevels;
    float mfScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once)
    static int mnMinX;
    static int mnMaxX;
    static int mnMinY;
    static int mnMaxY;

    static bool mbInitialComputations;


private:

    void UndistortKeyPoints();
    void ComputeImageBounds();

    // Call UpdatePoseMatrices(), before using
    cv::Mat mOw;
    cv::Mat mRcw;
    cv::Mat mtcw;
};

}// namespace ORB_SLAM

#endif // FRAME_H
