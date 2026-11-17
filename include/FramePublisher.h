// SPDX-FileCopyrightText: 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
// SPDX-FileCopyrightText: 2015-2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef FRAMEPUBLISHER_H
#define FRAMEPUBLISHER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"
#include "ORB_SLAM_export.h"


#include<opencv2/core.hpp>

#include <mutex>


namespace ORB_SLAM
{

class Tracking;

class ORB_SLAM_EXPORT FramePublisher
{
public:
    FramePublisher();

    void Update(Tracking *pTracker);

    void SetMap(Map* pMap);
    cv::Mat DrawFrame();

protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    cv::Mat mIm;
    vector<cv::KeyPoint> mvCurrentKeys;

    vector<bool> mvbOutliers;

    vector<MapPoint*> mvpMatchedMapPoints;
    int mnTracked;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;


    int mState;

    Map* mpMap;

    std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif // FRAMEPUBLISHER_H
