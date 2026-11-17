/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* Copyright (C) 2015-2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"
#include "ORB_SLAM_export.h"

#include <condition_variable>
#include <mutex>
#include <stop_token>

#include "KeyFrameDatabase.h"

#include <g2o/types/sim3/types_seven_dof_expmap.h>

namespace ORB_SLAM
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;


class ORB_SLAM_EXPORT LoopClosing
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;

public:

    LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc);

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    void Run(std::stop_token stopToken);

    void InsertKeyFrame(KeyFrame *pKF);

    void RequestReset();

protected:

    bool CheckNewKeyFrames();

    bool DetectLoop();

    bool ComputeSim3();

    void SearchAndFuse(KeyFrameAndPose &CorrectedPosesMap);

    void CorrectLoop(std::stop_token stopToken);

    bool ResetRequested();

    //! Wakes the worker up to look for work.
    void WakeUp();

    void ResetIfRequested();

    //! Drops the pointers to erased keyframes and map points kept between keyframes.
    void DropErasedObjects();

    bool mbResetRequested;
    std::mutex mMutexReset;

    Map* mpMap;
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;

    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    std::condition_variable_any processNext;
    std::condition_variable resetDone;
    std::mutex processMutex;

    std::vector<float> mvfLevelSigmaSquare;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFrame* mpCurrentKF{nullptr};
    KeyFrame* mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;
    double mScale_cw;

    long unsigned int mLastLoopKFid;

    // Keeps erased objects allocated until the worker passed quiescent points.
    Map::Participant mParticipant;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
