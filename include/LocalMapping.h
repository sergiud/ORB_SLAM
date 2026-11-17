// SPDX-FileCopyrightText: 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
// SPDX-FileCopyrightText: 2015-2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <stop_token>

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "ORB_SLAM_export.h"

namespace ORB_SLAM
{

class Tracking;
class LoopClosing;
class Map;

class ORB_SLAM_EXPORT LocalMapping
{
public:
    LocalMapping(Map* pMap);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    void Run(std::stop_token stopToken);

    bool InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestPause();
    void RequestReset();

    bool Pause();

    void Resume();

    bool WaitUntilPaused(std::stop_token stopToken);

    bool isPaused();

    bool pauseRequested();

    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);

    void InterruptBA();

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    void ResetIfRequested();
    bool ResetRequested();

    //! Drops the pointers to erased keyframes and map points kept between keyframes.
    void DropErasedObjects();

    //! Wakes the worker up to look for work.
    void WakeUp();

    bool mbResetRequested;
    std::mutex mMutexReset;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame{nullptr};

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    // Keeps erased objects allocated until the worker passed quiescent points.
    Map::Participant mParticipant;
    std::condition_variable_any processNext;
    std::condition_variable resetDone;
    std::mutex processMutex;

    std::atomic<bool> mbAbortBA;
    std::atomic<bool> mbPaused;
    std::atomic<bool> mbPauseRequested;
    std::mutex mMutexPause;
    std::condition_variable_any pausedChanged;

    std::atomic<bool> mbAcceptKeyFrames;
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
