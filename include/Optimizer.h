// SPDX-FileCopyrightText: 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
// SPDX-FileCopyrightText: 2015-2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <atomic>

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORB_SLAM_export.h"

#include <g2o/types/sim3/types_seven_dof_expmap.h>

namespace ORB_SLAM
{

class LoopClosing;

class ORB_SLAM_EXPORT Optimizer
{
public:
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP, int nIterations = 5, bool *pbStopFlag=NULL);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL);
    /**
     * Optimizes the poses of the keyframe and its covisible keyframes and
     * the map points they see.
     *
     * @param pKF The keyframe whose neighborhood is optimized.
     * @param pbStopFlag Another thread may set the flag to stop the
     *        optimization after its current iteration.
     */
    void static LocalBundleAdjustment(KeyFrame* pKF, const std::atomic<bool>* pbStopFlag=NULL);
    int static PoseOptimization(Frame* pFrame);

    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF, g2o::Sim3 &Scurw,
                                       LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       std::map<KeyFrame*, set<KeyFrame*> > &LoopConnections);


    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, float th2 = 10);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
