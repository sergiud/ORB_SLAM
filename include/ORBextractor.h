// SPDX-FileCopyrightText: 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
// SPDX-FileCopyrightText: 2015-2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/core.hpp>

#include "ORB_SLAM_export.h"

namespace ORB_SLAM
{

class ORB_SLAM_EXPORT ORBextractor
{
public:

    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures = 1000, float scaleFactor = 1.2f, int nlevels = 8, int scoreType=FAST_SCORE, int fastTh = 20);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}


protected:

    void ComputePyramid(cv::Mat image, cv::Mat Mask=cv::Mat());
    //! Columns and rows of the cells that a pyramid level is divided into.
    struct Grid
    {
        int cols;
        int rows;
    };

    /**
     * Returns the cells that spread the features of a pyramid level over its
     * image.
     *
     * @param features The number of features the level retains.
     * @param imageRatio The width of the image divided by its height.
     */
    static Grid CellGrid(int features, float imageRatio);

    void ComputeKeyPoints(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);

    std::vector<cv::Point> pattern;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int scoreType;
    int fastTh;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;

    std::vector<cv::Mat> mvImagePyramid;
    std::vector<cv::Mat> mvMaskPyramid;

};

} //namespace ORB_SLAM

#endif

