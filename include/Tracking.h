// SPDX-FileCopyrightText: 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
// SPDX-FileCopyrightText: 2015-2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core.hpp>


#include "FramePublisher.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "ORB_SLAM_export.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>


namespace ORB_SLAM
{

class FramePublisher;
class Map;
class LocalMapping;
class LoopClosing;

class ORB_SLAM_EXPORT Tracking
{

public:
    Tracking(ORBVocabulary* pVoc, FramePublisher *pFramePublisher, Map *pMap,
        const cv::Mat& K, const cv::Mat& distCoeffs, double fps,
        int nFeatures = 1000,
        float fScaleFactor = 1.2f,
        int nLevels = 8,
        int fastTh = 20,
        int Score = 1,
        bool UseMotionModel = true
        );
    Tracking(ORBVocabulary* pVoc, FramePublisher* pFramePublisher, Map* pMap, string strSettingPath);

    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        INITIALIZING=2,
        WORKING=3,
        LOST=4
    };

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetKeyFrameDatabase(KeyFrameDatabase* pKFDB);

    void ForceRelocalisation();

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Current Frame
    Frame mCurrentFrame;

    // Initialization Variables
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;


    /**
     * Tracks the camera in an image.
     *
     * @param image The grayscale image.
     * @param timestamp The time in seconds at which the image was taken,
     *        which the keyframes made from it carry.
     */
    void Track(const cv::Mat& image, double timestamp);

    eTrackingState GetState() const;
    bool IsTracking() const;
    bool IsLost() const;
    bool IsInitializing() const;
    bool IsRunning() const;
    cv::Mat GetCurrentCameraPose() const;
    //! Tracked keypoints used for visualization.
    std::vector<cv::KeyPoint> GetKeyPoints() const;

protected:

    //! Tracks the camera in an image, as @ref Track describes.
    void TrackImage(const cv::Mat& image, double timestamp);
    //! Drops the pointers to erased keyframes and map points kept between images.
    void DropErasedObjects();

    void FirstInitialization();
    void Initialize();
    void CreateInitialMap(cv::Mat &Rcw, cv::Mat &tcw);

    void Reset();

    bool TrackPreviousFrame();
    bool TrackWithMotionModel();

    bool RelocalisationRequested();
    bool Relocalisation();

    void UpdateReference();
    void UpdateReferencePoints();
    void UpdateReferenceKeyFrames();

    bool TrackLocalMap();
    void SearchReferencePointsInFrustum();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();


    //Other Thread Pointers
    LocalMapping* mpLocalMapper{nullptr};
    LoopClosing* mpLoopClosing{nullptr};

    //ORB
    std::unique_ptr<ORBextractor> mpORBextractor;
    std::unique_ptr<ORBextractor> mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB{nullptr};

    // Initalization
    std::unique_ptr<Initializer> mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF{nullptr};
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;

    //Publisher
    FramePublisher* mpFramePublisher;

    //Map
    Map* mpMap;
    // Keeps erased objects allocated until the tracker passed quiescent
    // points, if the tracker has a map.
    std::optional<Map::Participant> mParticipant;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame{nullptr};
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Mutex
    std::mutex mMutexTrack;

    //Is relocalisation requested by an external thread? (loop closing)
    std::atomic<bool> mbForceRelocalisation;

    //Motion Model
    bool mbMotionModel;
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

};

} //namespace ORB_SLAM

#endif // TRACKING_H
