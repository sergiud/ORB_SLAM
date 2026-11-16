/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

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
    void RequestStop();
    void RequestReset();

    bool Stop();

    void Release();

    bool WaitUntilStopped(std::stop_token stopToken);

    bool isStopped();

    bool stopRequested();

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
    bool mbResetRequested;
    std::mutex mMutexReset;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;
    std::condition_variable_any processNext;
    std::condition_variable resetDone;
    std::mutex processMutex;

    bool mbAbortBA;
    std::atomic<bool> mbStopped;
    std::atomic<bool> mbStopRequested;
    std::mutex mMutexStop;
    std::condition_variable_any stoppedChanged;

    std::atomic<bool> mbAcceptKeyFrames;
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
