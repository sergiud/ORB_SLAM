// SPDX-FileCopyrightText: 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
// SPDX-FileCopyrightText: 2015-2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "ORB_SLAM_export.h"

#include<set>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

namespace ORB_SLAM
{

class MapPoint;
class KeyFrame;

/**
 * Keyframes and map points of a map.
 *
 * The map owns the keyframes and map points added to it and frees them when
 * it is cleared or destroyed. An erased keyframe or map point leaves the map
 * but stays allocated until no thread can refer to it anymore, as
 * @ref Reclaim describes.
 */
class ORB_SLAM_EXPORT Map
{
public:
    /**
     * Thread that keeps pointers to keyframes and map points of the map
     * between its quiescent points.
     *
     * At a quiescent point, the thread refers to no erased keyframe or map
     * point, so that it neither reads one later nor stores one where another
     * thread reads it.
     */
    class ORB_SLAM_EXPORT Participant
    {
    public:
        //! Registers a thread that keeps pointers to objects of @p map.
        explicit Participant(Map& map);
        ~Participant();

        Participant(const Participant&) = delete;
        Participant& operator=(const Participant&) = delete;

        //! States that the thread refers to no erased object.
        void Quiesce();

    private:
        friend class Map;

        Map& mMap;
        std::atomic<std::uint64_t> mEpoch;
    };

    Map();
    ~Map();

    Map(const Map&) = delete;
    Map& operator=(const Map&) = delete;

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    int MapPointsInMap();
    int KeyFramesInMap();

    void SetFlagAfterBA();
    bool isMapUpdated();
    void ResetUpdated();

    unsigned int GetMaxKFid();

    void clear();

    /**
     * Frees the erased keyframes and map points that no thread can refer to
     * anymore.
     *
     * Erased objects are freed in batches. Once every participant passed a
     * quiescent point after the erasure of a batch, no participant obtains
     * an object of the batch anew, and the map removes the references to it
     * that keyframes and map points kept through concurrent updates. Once
     * every participant passed a quiescent point after that removal as
     * well, no thread refers to the batch and the map frees it. Each call
     * advances the batches as far as the quiescent points allow and returns
     * without waiting.
     */
    void Reclaim();

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    unsigned int mnMaxKFid;

    std::mutex mMutexMap;
    bool mbMapUpdated;

private:
    //! Keyframes and map points erased before the start of a grace period.
    struct ErasedObjects
    {
        std::vector<KeyFrame*> keyFrames;
        std::vector<MapPoint*> mapPoints;
        //! Epoch that every participant reaches after the grace period.
        std::uint64_t epoch = 0;

        bool empty() const;
    };

    //! Returns whether every participant passed a quiescent point at or after @p epoch.
    bool QuiescedSince(std::uint64_t epoch) const;
    //! Removes the references to @p erased from the objects that remain.
    void RemoveReferences(const ErasedObjects& erased);
    //! Frees the keyframes and map points of @p objects and forgets them.
    static void Free(ErasedObjects& objects);

    // Erased since the last batch started its first grace period, guarded
    // by the map mutex.
    ErasedObjects mErased;
    // Guarded by the reclamation mutex, which the map mutex nests in.
    ErasedObjects mUnreferencing;
    ErasedObjects mFreeing;
    std::vector<Participant*> mParticipants;
    std::mutex mMutexReclaim;
    std::atomic<std::uint64_t> mEpoch{0};
};

} //namespace ORB_SLAM

#endif // MAP_H
