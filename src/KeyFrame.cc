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

#include "KeyFrame.h"
#include "Converter.h"
#ifdef HAVE_ROS
#include <ros/ros.h>
#endif // HAVE_ROS

namespace ORB_SLAM
{

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mfGridElementWidthInv(F.mfGridElementWidthInv),
    mfGridElementHeightInv(F.mfGridElementHeightInv), mnTrackReferenceForFrame(0),mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnRelocQuery(0),fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), mBowVec(F.mBowVec),
    im(F.im), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX), mnMaxY(F.mnMaxY), mK(F.mK),
    mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn), mDescriptors(F.mDescriptors.clone()),
    mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB), mpORBvocabulary(F.mpORBvocabulary), mFeatVec(F.mFeatVec),
    mbFirstConnection(true), mpParent(NULL), mbNotErase(false), mbToBeErased(false), mbBad(false),
    mnScaleLevels(F.mnScaleLevels), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mpMap(pMap)
{
    mnId=nNextId++;

    mnGridCols=FRAME_GRID_COLS;
    mnGridRows=FRAME_GRID_ROWS;
    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);
}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const cv::Mat &Rcw,const cv::Mat &tcw)
{
    std::scoped_lock lock{mMutexPose};
    Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
    tcw.copyTo(Tcw.col(3).rowRange(0,3));

    Ow=-Rcw.t()*tcw;
}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    std::scoped_lock lock{mMutexPose};
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    Ow = -Rcw.t()*tcw;
}

cv::Mat KeyFrame::GetPose()
{
    std::scoped_lock lock{mMutexPose};
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    std::scoped_lock lock{mMutexPose};
    cv::Mat Twc = cv::Mat::eye(4,4,Tcw.type());
    cv::Mat Rwc = (Tcw.rowRange(0,3).colRange(0,3)).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    twc.copyTo(Twc.rowRange(0,3).col(3));
    return Twc.clone();
}

cv::Mat KeyFrame::GetProjectionMatrix()
{
    std::scoped_lock lock{mMutexPose};
    return mK*Tcw.rowRange(0,3);
}

cv::Mat KeyFrame::GetCameraCenter()
{
    std::scoped_lock lock{mMutexPose};
    return Ow.clone();
}

cv::Mat KeyFrame::GetRotation()
{
    std::scoped_lock lock{mMutexPose};
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    std::scoped_lock lock{mMutexPose};
    return Tcw.rowRange(0,3).col(3).clone();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        std::scoped_lock lock{mMutexConnections};
        // An erased keyframe takes part in no covisibility graph. The
        // erasure marks a keyframe before it removes the connections of
        // others to it under their locks, so a connection either sees the
        // mark here or is removed afterwards.
        if(mbBad || pKF->mbBad)
            return;
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    std::scoped_lock lock{mMutexConnections};
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    std::scoped_lock lock{mMutexConnections};
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    std::scoped_lock lock{mMutexConnections};
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    std::scoped_lock lock{mMutexConnections};
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    std::scoped_lock lock{mMutexConnections};

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<KeyFrame*>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    std::scoped_lock lock{mMutexConnections};
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    std::scoped_lock lock{mMutexFeatures};
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    std::scoped_lock lock{mMutexFeatures};
    mvpMapPoints[idx]=NULL;
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    // The map point locks itself, so its index is looked up before the lock.
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx>=0)
    {
        std::scoped_lock lock{mMutexFeatures};
        mvpMapPoints[idx]=NULL;
    }
}


void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
    std::scoped_lock lock{mMutexFeatures};
    mvpMapPoints[idx]=pMP;
}

set<MapPoint*> KeyFrame::GetMapPoints()
{
    std::scoped_lock lock{mMutexFeatures};
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints()
{
    std::scoped_lock lock{mMutexFeatures};

    int nPoints=0;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(mvpMapPoints[i])
            nPoints++;
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    std::scoped_lock lock{mMutexFeatures};
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    std::scoped_lock lock{mMutexFeatures};
    return mvpMapPoints[idx];
}

cv::KeyPoint KeyFrame::GetKeyPointUn(const size_t &idx) const
{
    return mvKeysUn[idx];
}

int KeyFrame::GetKeyPointScaleLevel(const size_t &idx) const
{
    return mvKeysUn[idx].octave;
}

cv::Mat KeyFrame::GetDescriptor(const size_t &idx)
{
    // The descriptors never change after construction, so the row is
    // shared rather than copied.
    return mDescriptors.row(idx);
}

cv::Mat KeyFrame::GetDescriptors()
{
    return mDescriptors.clone();
}

vector<cv::KeyPoint> KeyFrame::GetKeyPoints() const
{
    return mvKeys;
}

vector<cv::KeyPoint> KeyFrame::GetKeyPointsUn() const
{
    return mvKeysUn;
}

cv::Mat KeyFrame::GetCalibrationMatrix() const
{
    return mK.clone();
}

DBoW2::FeatureVector KeyFrame::GetFeatureVector()
{
    std::scoped_lock lock{mMutexFeatures};
    return mFeatVec;
}

DBoW2::BowVector KeyFrame::GetBowVector()
{
    std::scoped_lock lock{mMutexFeatures};
    return mBowVec;
}

cv::Mat KeyFrame::GetImage()
{
    std::scoped_lock lock{mMutexImage};
    return im.clone();
}

void KeyFrame::UpdateConnections()
{
    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;

    {
        std::scoped_lock lockMPs{mMutexFeatures};
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
    }

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    KeyFrame* pNewParent = NULL;

    {
        std::scoped_lock lockCon{mMutexConnections};

        if(mbBad)
            return;

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mnId!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            pNewParent = mpParent;
            mbFirstConnection = false;
        }

    }

    // The neighbors connect to this keyframe only once it holds its own
    // connections, so that an erasure that refused those leaves no neighbor
    // connected to it.
    for(size_t i=0; i<vPairs.size(); i++)
        vPairs[i].second->AddConnection(this,vPairs[i].first);

    // A keyframe locks no other keyframe while it holds its own lock, which
    // keeps keyframes that update each other from deadlocking.
    if(pNewParent)
        pNewParent->AddChild(this);
}

void KeyFrame::AddChild(KeyFrame *pKF)
{
    std::scoped_lock lockCon{mMutexConnections};
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    std::scoped_lock lockCon{mMutexConnections};
    mspChildrens.erase(pKF);
}


void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    {
        std::scoped_lock lockCon{mMutexConnections};
        mpParent = pKF;
    }

    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    std::scoped_lock lockCon{mMutexConnections};
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    std::scoped_lock lockCon{mMutexConnections};
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    std::scoped_lock lockCon{mMutexConnections};
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    std::scoped_lock lockCon{mMutexConnections};
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    std::scoped_lock lockCon{mMutexConnections};
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    std::scoped_lock lock{mMutexConnections};
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        std::scoped_lock lock{mMutexConnections};
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}


void KeyFrame::SetBadFlag()
{
    map<KeyFrame*,int> connections;
    vector<MapPoint*> vpMapPoints;
    set<KeyFrame*> sChildren;
    KeyFrame* pParent;

    {
        std::scoped_lock lock{mMutexConnections, mMutexFeatures};
        if(mnId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }

        // Other keyframes neither connect to nor adopt an erased keyframe,
        // so what is taken here is all there is to undo. The work below
        // locks one other keyframe at a time, which keeps erasures of
        // neighboring keyframes from deadlocking.
        mbBad = true;
        connections.swap(mConnectedKeyFrameWeights);
        mvpOrderedConnectedKeyFrames.clear();
        mvOrderedWeights.clear();
        vpMapPoints = mvpMapPoints;
        sChildren.swap(mspChildrens);
        pParent = mpParent;
    }

    for(map<KeyFrame*,int>::iterator mit = connections.begin(), mend=connections.end(); mit!=mend; mit++)
        mit->first->EraseConnection(this);

    for(size_t i=0; i<vpMapPoints.size(); i++)
        if(vpMapPoints[i])
            vpMapPoints[i]->EraseObservation(this);

    // Update Spanning Tree
    set<KeyFrame*> sParentCandidates;
    sParentCandidates.insert(pParent);

    // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
    // Include that children as new parent candidate for the rest
    while(!sChildren.empty())
    {
        bool bContinue = false;

        int max = -1;
        KeyFrame* pC;
        KeyFrame* pP;

        for(set<KeyFrame*>::iterator sit=sChildren.begin(), send=sChildren.end(); sit!=send; sit++)
        {
            KeyFrame* pKF = *sit;
            if(pKF->isBad())
                continue;

            // Check if a parent candidate is connected to the keyframe
            vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
            for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
            {
                for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                {
                    if(vpConnected[i]->mnId == (*spcit)->mnId)
                    {
                        int w = pKF->GetWeight(vpConnected[i]);
                        if(w>max)
                        {
                            pC = pKF;
                            pP = vpConnected[i];
                            max = w;
                            bContinue = true;
                        }
                    }
                }
            }
        }

        if(bContinue)
        {
            pC->ChangeParent(pP);
            sParentCandidates.insert(pC);
            sChildren.erase(pC);
        }
        else
            break;
    }

    // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
    for(set<KeyFrame*>::iterator sit=sChildren.begin(); sit!=sChildren.end(); sit++)
    {
        (*sit)->ChangeParent(pParent);
    }

    pParent->EraseChild(this);

    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad()
{
    std::scoped_lock lock{mMutexConnections};
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        std::scoped_lock lock{mMutexConnections};
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(mvKeysUn.size());

    int nMinCellX = floor((x-mnMinX-r)*mfGridElementWidthInv);
    nMinCellX = max(0,nMinCellX);
    if(nMinCellX>=mnGridCols)
        return vIndices;

    int nMaxCellX = ceil((x-mnMinX+r)*mfGridElementWidthInv);
    nMaxCellX = min(mnGridCols-1,nMaxCellX);
    if(nMaxCellX<0)
        return vIndices;

    int nMinCellY = floor((y-mnMinY-r)*mfGridElementHeightInv);
    nMinCellY = max(0,nMinCellY);
    if(nMinCellY>=mnGridRows)
        return vIndices;

    int nMaxCellY = ceil((y-mnMinY+r)*mfGridElementHeightInv);
    nMaxCellY = min(mnGridRows-1,nMaxCellY);
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(abs(kpUn.pt.x-x)<=r && abs(kpUn.pt.y-y)<=r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

float KeyFrame::ComputeSceneMedianDepth(int q)
{
    vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
    std::scoped_lock lock{mMutexFeatures};
    std::scoped_lock lock2{mMutexPose};
    vpMapPoints = mvpMapPoints;
    Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(mvpMapPoints.size());
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(size_t i=0; i<mvpMapPoints.size(); i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

} //namespace ORB_SLAM
