// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef ORB_SLAM_TESTS_FRAMES_H
#define ORB_SLAM_TESTS_FRAMES_H

#include <cstddef>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "Frame.h"

namespace ORB_SLAM::tests {

//! Scale pyramid of a frame.
struct Pyramid
{
    float scaleFactor = 1;
    int levels = 1;
};

//! Number of bytes of an ORB descriptor.
inline constexpr int descriptorBytes = 32;

/**
 * Sets the camera that all frames share, as the first frame made from an
 * image does.
 *
 * @param K The camera intrinsic matrix.
 * @param width The width of the undistorted image in pixels.
 * @param height The height of the undistorted image in pixels.
 */
inline void setCamera(const cv::Mat& K, int width, int height)
{
    Frame::fx = K.at<float>(0, 0);
    Frame::fy = K.at<float>(1, 1);
    Frame::cx = K.at<float>(0, 2);
    Frame::cy = K.at<float>(1, 2);
    Frame::mnMinX = 0;
    Frame::mnMaxX = width;
    Frame::mnMinY = 0;
    Frame::mnMaxY = height;
    Frame::mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / width;
    Frame::mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / height;
    Frame::mbInitialComputations = false;
}

/**
 * Creates a frame from undistorted keypoints instead of an image.
 *
 * The frame needs neither image, extractor nor vocabulary. Its keypoints
 * see no map points, and its scale pyramid follows @p pyramid as the
 * extractor would build it.
 *
 * @param Tcw The pose of the frame, mapping world to camera coordinates.
 * @param K The camera intrinsic matrix.
 * @param keyPoints The undistorted keypoints, also used as the original ones.
 * @param pyramid The scale pyramid the keypoints were detected in.
 * @param descriptors The descriptor of each keypoint, one per row. Without
 *        them, every keypoint has a descriptor of zeros.
 *
 * Once @ref setCamera set the camera, the keypoints are also assigned to the
 * grid cells of the frame, which the searches in an area use.
 */
inline Frame makeFrame(const cv::Mat& Tcw, const cv::Mat& K = cv::Mat{},
                       std::vector<cv::KeyPoint> keyPoints = {},
                       Pyramid pyramid = {}, const cv::Mat& descriptors = {})
{
    Frame frame;
    frame.N = static_cast<int>(keyPoints.size());
    frame.mnId = 0;
    frame.mTimeStamp = 0;
    frame.mK = K;
    frame.mvKeys = keyPoints;
    frame.mvKeysUn = std::move(keyPoints);
    frame.mvpMapPoints.assign(frame.mvKeysUn.size(), nullptr);
    frame.mvbOutlier.assign(frame.mvKeysUn.size(), false);
    frame.mnScaleLevels = pyramid.levels;
    frame.mfScaleFactor = pyramid.scaleFactor;

    float scale = 1;

    for (int level = 0; level < pyramid.levels; ++level) {
        frame.mvScaleFactors.push_back(scale);
        frame.mvLevelSigma2.push_back(scale * scale);
        frame.mvInvLevelSigma2.push_back(1 / (scale * scale));
        scale *= pyramid.scaleFactor;
    }

    frame.mDescriptors = descriptors.empty()
        ? cv::Mat::zeros(frame.N, descriptorBytes, CV_8U)
        : descriptors.clone();

    if (!Frame::mbInitialComputations) {
        for (std::size_t i = 0; i < frame.mvKeysUn.size(); ++i) {
            int column = 0;
            int row = 0;

            if (frame.PosInGrid(frame.mvKeysUn[i], column, row)) {
                frame.mGrid[column][row].push_back(i);
            }
        }
    }

    frame.mTcw = Tcw.clone();

    return frame;
}

} // namespace ORB_SLAM::tests

#endif // ORB_SLAM_TESTS_FRAMES_H
