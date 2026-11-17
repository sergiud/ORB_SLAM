// SPDX-FileCopyrightText: 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
// SPDX-FileCopyrightText: 2015-2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "MapPoint.h"
#include "ORBmatcher.h"

#include <algorithm>
#include <cmath>
#include <vector>


namespace ORB_SLAM
{

long unsigned int MapPoint::nNextId=0;


MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnTrackReferenceForFrame(0), mnLastFrameSeen(0), mnBALocalForKF(0),
    mnLoopPointForKF(0), mnCorrectedByKF(0),mnCorrectedReference(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1),
    mbBad(false), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mnId=nNextId++;
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    std::scoped_lock lock{mMutexPos};
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    std::scoped_lock lock{mMutexPos};
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    std::scoped_lock lock{mMutexPos};
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
     std::scoped_lock lock{mMutexFeatures};
     return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    // The map frees an erased keyframe once its references are removed, so
    // none is taken after that.
    if(pKF->isBad())
    {
        return;
    }

    std::scoped_lock lock{mMutexFeatures};
    mObservations[pKF]=idx;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        std::scoped_lock lock{mMutexFeatures};
        if(mObservations.count(pKF))
        {
            mObservations.erase(pKF);

            // A point without observations has no reference keyframe.
            if(mpRefKF==pKF)
                mpRefKF=mObservations.empty() ? NULL : mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(mObservations.size()<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    std::scoped_lock lock{mMutexFeatures};
    return mObservations;
}

int MapPoint::Observations()
{
    std::scoped_lock lock{mMutexFeatures};
    return mObservations.size();
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        std::scoped_lock lock1{mMutexFeatures};
        std::scoped_lock lock2{mMutexPos};
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }

    mpMap->EraseMapPoint(this);
}

void MapPoint::RemoveReferences(const std::unordered_set<KeyFrame*>& keyFrames)
{
    std::scoped_lock lock{mMutexFeatures};

    std::erase_if(mObservations, [&keyFrames](const std::pair<KeyFrame* const, size_t>& observation) {
        return keyFrames.contains(observation.first);
    });

    if(keyFrames.contains(mpRefKF))
    {
        mpRefKF=mObservations.empty() ? nullptr : mObservations.begin()->first;
    }
}

void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    map<KeyFrame*,size_t> obs;
    {
        std::scoped_lock lock1{mMutexFeatures};
        std::scoped_lock lock2{mMutexPos};
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }

    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);

}

bool MapPoint::isBad()
{
    std::scoped_lock lock{mMutexFeatures};
    std::scoped_lock lock2{mMutexPos};
    return mbBad;
}

void MapPoint::IncreaseVisible()
{
    std::scoped_lock lock{mMutexFeatures};
    mnVisible++;
}

void MapPoint::IncreaseFound()
{
    std::scoped_lock lock{mMutexFeatures};
    mnFound++;
}

float MapPoint::GetFoundRatio()
{
    std::scoped_lock lock{mMutexFeatures};
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        std::scoped_lock lock1{mMutexFeatures};
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->GetDescriptor(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    std::vector<std::vector<float> > Distances(N, std::vector<float>(N, 0.0f));
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i].begin(), Distances[i].end());
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        std::scoped_lock lock{mMutexFeatures};
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    std::scoped_lock lock{mMutexFeatures};
    // A new descriptor replaces the matrix rather than writing into it, so
    // the matrix is shared rather than copied.
    return mDescriptor;
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    std::scoped_lock lock{mMutexFeatures};
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    std::scoped_lock lock{mMutexFeatures};
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        std::scoped_lock lock1{mMutexFeatures};
        std::scoped_lock lock2{mMutexPos};
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = Pos - Owi;
        normal = normal + normali/cv::norm(normali);
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->GetKeyPointScaleLevel(observations[pRefKF]);
    const float levelScaleFactor =  pRefKF->GetScaleFactor(level);
    const int nLevels = pRefKF->GetScaleLevels();

    {
        std::scoped_lock lock3{mMutexPos};
        // Coarser levels detect the point closer to the camera, down to the
        // coarsest level, and the finest level detects it farthest away.
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->GetScaleFactor(nLevels-1);
        // Viewing directions that cancel each other have no mean direction.
        const double length = cv::norm(normal);
        if(length>0)
            mNormalVector = normal/length;
    }
}

int MapPoint::PredictScale(float dist, const std::vector<float>& scaleFactors)
{
    const int maxLevel = static_cast<int>(scaleFactors.size())-1;

    if(maxLevel<1)
        return 0;

    // The point appears one level coarser for each scale factor the camera
    // comes closer than the largest distance.
    const float ratio = GetMaxDistanceInvariance()/dist;
    const long level = std::lround(std::log(ratio)/std::log(scaleFactors[1]));

    return static_cast<int>(std::clamp(level, 0L, static_cast<long>(maxLevel)));
}

float MapPoint::GetMinDistanceInvariance()
{
    std::scoped_lock lock{mMutexPos};
    return mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    std::scoped_lock lock{mMutexPos};
    return mfMaxDistance;
}

} //namespace ORB_SLAM
