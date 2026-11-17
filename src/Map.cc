/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* Copyright (C) 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
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

#include "Map.h"

#include <algorithm>
#include <iterator>
#include <unordered_set>
#include <utility>

namespace ORB_SLAM
{

Map::Participant::Participant(Map& map)
    : mMap{map}
    , mEpoch{map.mEpoch.load()}
{
    std::scoped_lock lock{mMap.mMutexReclaim};
    mMap.mParticipants.push_back(this);
}

Map::Participant::~Participant()
{
    std::scoped_lock lock{mMap.mMutexReclaim};
    std::erase(mMap.mParticipants, this);
}

void Map::Participant::Quiesce()
{
    mEpoch.store(mMap.mEpoch.load());
}

bool Map::ErasedObjects::empty() const
{
    return keyFrames.empty() && mapPoints.empty();
}

Map::Map()
{
    mbMapUpdated= false;
    mnMaxKFid = 0;
}

Map::~Map()
{
    clear();
}

bool Map::QuiescedSince(std::uint64_t epoch) const
{
    return std::ranges::all_of(mParticipants, [epoch](const Participant* participant) {
        return participant->mEpoch.load() >= epoch;
    });
}

void Map::Reclaim()
{
    std::scoped_lock lock{mMutexReclaim};

    if(!mFreeing.empty() && QuiescedSince(mFreeing.epoch))
    {
        Free(mFreeing);
    }

    // Participants that passed a quiescent point after the erasure obtain no
    // object of the batch anew, since they find each one erased. What the
    // objects that remain still refer to of the batch is removed here, and
    // what a participant read before that removal it gives up at its next
    // quiescent point.
    if(mFreeing.empty() && !mUnreferencing.empty() && QuiescedSince(mUnreferencing.epoch))
    {
        RemoveReferences(mUnreferencing);
        mFreeing = std::exchange(mUnreferencing, {});
        mFreeing.epoch = ++mEpoch;
    }

    if(mUnreferencing.empty())
    {
        {
            std::scoped_lock lockMap{mMutexMap};
            std::swap(mUnreferencing, mErased);
        }

        if(!mUnreferencing.empty())
        {
            mUnreferencing.epoch = ++mEpoch;
        }
    }
}

void Map::RemoveReferences(const ErasedObjects& erased)
{
    const std::unordered_set<KeyFrame*> keyFrames(erased.keyFrames.begin(), erased.keyFrames.end());
    const std::unordered_set<MapPoint*> mapPoints(erased.mapPoints.begin(), erased.mapPoints.end());

    // Objects erased later may still be read until their own batch is freed,
    // so their references are removed as well.
    std::vector<KeyFrame*> vpKFs;
    std::vector<MapPoint*> vpMPs;

    {
        std::scoped_lock lock{mMutexMap};
        vpKFs.assign(mspKeyFrames.begin(), mspKeyFrames.end());
        vpKFs.insert(vpKFs.end(), mErased.keyFrames.begin(), mErased.keyFrames.end());
        vpMPs.assign(mspMapPoints.begin(), mspMapPoints.end());
        vpMPs.insert(vpMPs.end(), mErased.mapPoints.begin(), mErased.mapPoints.end());
        std::erase_if(mvpReferenceMapPoints, [&mapPoints](MapPoint* pMP) {
            return mapPoints.contains(pMP);
        });
    }

    for(KeyFrame* pKF : vpKFs)
    {
        pKF->RemoveReferences(keyFrames, mapPoints);
    }

    for(MapPoint* pMP : vpMPs)
    {
        pMP->RemoveReferences(keyFrames);
    }

    for(KeyFrame* pKF : erased.keyFrames)
    {
        pKF->EraseFromDatabase();
    }
}

void Map::Free(ErasedObjects& objects)
{
    for(MapPoint* pMP : objects.mapPoints)
    {
        delete pMP;
    }

    for(KeyFrame* pKF : objects.keyFrames)
    {
        delete pKF;
    }

    objects = {};
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    std::scoped_lock lock{mMutexMap};
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
    mbMapUpdated=true;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    std::scoped_lock lock{mMutexMap};
    mspMapPoints.insert(pMP);
    mbMapUpdated=true;
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    std::scoped_lock lock{mMutexMap};
    // A map point erased again was taken for freeing already.
    if(mspMapPoints.erase(pMP))
    {
        mErased.mapPoints.push_back(pMP);
    }
    mbMapUpdated=true;
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    std::scoped_lock lock{mMutexMap};
    if(mspKeyFrames.erase(pKF))
    {
        mErased.keyFrames.push_back(pKF);
    }
    mbMapUpdated=true;
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    std::vector<MapPoint*> vpGood;
    vpGood.reserve(vpMPs.size());
    std::ranges::copy_if(vpMPs, std::back_inserter(vpGood), [](MapPoint* pMP) {
        return !pMP->isBad();
    });

    std::scoped_lock lock{mMutexMap};
    mvpReferenceMapPoints = std::move(vpGood);
    mbMapUpdated=true;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    std::scoped_lock lock{mMutexMap};
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    std::scoped_lock lock{mMutexMap};
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

int Map::MapPointsInMap()
{
    std::scoped_lock lock{mMutexMap};
    return mspMapPoints.size();
}

int Map::KeyFramesInMap()
{
    std::scoped_lock lock{mMutexMap};
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    std::scoped_lock lock{mMutexMap};
    return mvpReferenceMapPoints;
}

bool Map::isMapUpdated()
{
    std::scoped_lock lock{mMutexMap};
    return mbMapUpdated;
}

void Map::SetFlagAfterBA()
{
    std::scoped_lock lock{mMutexMap};
    mbMapUpdated=true;

}

void Map::ResetUpdated()
{
    std::scoped_lock lock{mMutexMap};
    mbMapUpdated=false;
}

unsigned int Map::GetMaxKFid()
{
    std::scoped_lock lock{mMutexMap};
    return mnMaxKFid;
}

void Map::clear()
{
    std::scoped_lock lock{mMutexReclaim, mMutexMap};

    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    Free(mErased);
    Free(mUnreferencing);
    Free(mFreeing);

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
}

} //namespace ORB_SLAM
