// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests of the frames that the tracker makes from images.

#include <algorithm>
#include <cstddef>
#include <format>
#include <string_view>
#include <vector>

#include <opencv2/core.hpp>

#include "Check.h"
#include "Frame.h"
#include "Frames.h"
#include "ORBextractor.h"

namespace {

using ORB_SLAM::tests::check;

constexpr int features = 1000;
constexpr float scaleFactor = 1.2F;
constexpr int levels = 8;
constexpr int fastThreshold = 20;
constexpr int fastScore = 1;
constexpr int width = 640;
constexpr int height = 480;
constexpr float focalLength = 500;
constexpr float radialDistortion = -0.3F;

cv::Mat cameraMatrix()
{
    return cv::Mat_<float>({3, 3}, {focalLength, 0, width / 2.0F,
                                    0, focalLength, height / 2.0F,
                                    0, 0, 1});
}

// A frame of an image without features, such as a covered lens, has an id of
// its own and the scale pyramid of its extractor, like any other frame.
void featurelessFrameIsComplete(const cv::Mat& distortion, std::string_view what)
{
    ORB_SLAM::ORBextractor extractor{features, scaleFactor, levels, fastScore,
                                     fastThreshold};
    const cv::Mat black = cv::Mat::zeros(height, width, CV_8U);

    ORB_SLAM::Frame first;
    ORB_SLAM::Frame second;

    try {
        first = ORB_SLAM::Frame{black, 0, &extractor, nullptr, cameraMatrix(),
                                distortion};
        second = ORB_SLAM::Frame{black, 0, &extractor, nullptr, cameraMatrix(),
                                 distortion};
    }
    catch (const cv::Exception& e) {
        check(false, std::format("{}: a featureless image makes a frame, got "
                                 "{:?}",
                                 what, e.what()));
        return;
    }

    check(second.mnId == first.mnId + 1,
          std::format("{}: consecutive frames have consecutive ids, got {} and "
                      "{}",
                      what, first.mnId, second.mnId));
    check(second.mnScaleLevels == levels &&
              second.mvScaleFactors.size() == static_cast<std::size_t>(levels) &&
              second.mvInvLevelSigma2.size() == static_cast<std::size_t>(levels),
          std::format("{}: the frame has the {} levels of its extractor, got {} "
                      "and {} scale factors",
                      what, levels, second.mnScaleLevels,
                      second.mvScaleFactors.size()));
}

// Exposes how the extractor divides a pyramid level into cells.
class GridExtractor : public ORB_SLAM::ORBextractor
{
public:
    using ORB_SLAM::ORBextractor::CellGrid;
};

// A level with fewer features than a cell holds still has a cell.
void levelHasCell(int levelFeatures, int imageWidth, int imageHeight)
{
    const float imageRatio = static_cast<float>(imageWidth) / imageHeight;
    const auto [cols, rows] = GridExtractor::CellGrid(levelFeatures, imageRatio);

    check(cols >= 1 && rows >= 1,
          std::format("{} features in a {}x{} image have a cell, got {}x{} "
                      "cells",
                      levelFeatures, imageWidth, imageHeight, cols, rows));
}

// The cells are about square, so that the features spread evenly over the
// image in both directions.
void cellsAreSquare(int levelFeatures, int imageWidth, int imageHeight)
{
    // A cell may be up to twice as wide as high, or the other way round.
    constexpr float maxElongation = 2;

    const float imageRatio = static_cast<float>(imageWidth) / imageHeight;
    const auto [cols, rows] = GridExtractor::CellGrid(levelFeatures, imageRatio);

    const float cellRatio = imageRatio * rows / cols;

    check(cellRatio <= maxElongation && cellRatio >= 1 / maxElongation,
          std::format("{} features in a {}x{} image have about square cells, "
                      "got {}x{} cells of ratio {}",
                      levelFeatures, imageWidth, imageHeight, cols, rows,
                      cellRatio));
}

// The cells hold about 5 features each.
void cellsHoldFewFeatures(int levelFeatures, int imageWidth, int imageHeight)
{
    constexpr float featuresPerCell = 5;
    // The number of cells may be off by up to this factor.
    constexpr float tolerance = 2;

    const float imageRatio = static_cast<float>(imageWidth) / imageHeight;
    const auto [cols, rows] = GridExtractor::CellGrid(levelFeatures, imageRatio);
    const float perCell = static_cast<float>(levelFeatures) / (cols * rows);

    check(perCell >= featuresPerCell / tolerance &&
              perCell <= featuresPerCell * tolerance,
          std::format("{} features in a {}x{} image fill {}x{} cells with "
                      "about {} each, got {}",
                      levelFeatures, imageWidth, imageHeight, cols, rows,
                      featuresPerCell, perCell));
}

// An extractor that asks few features of a richly textured image retains as
// many as it asks, including on its coarsest levels.
void retainsRequestedFeatures()
{
    constexpr int fewFeatures = 100;

    cv::Mat noise(height, width, CV_8U);
    cv::RNG generator;
    generator.fill(noise, cv::RNG::UNIFORM, 0, 256);

    ORB_SLAM::ORBextractor extractor{fewFeatures, scaleFactor, levels,
                                     fastScore, fastThreshold};
    std::vector<cv::KeyPoint> keyPoints;
    cv::Mat descriptors;
    extractor(noise, cv::Mat{}, keyPoints, descriptors);

    const auto coarsest = std::ranges::count_if(
        keyPoints, [](const cv::KeyPoint& k) { return k.octave == levels - 1; });

    check(keyPoints.size() == static_cast<std::size_t>(fewFeatures) &&
              coarsest > 0,
          std::format("{} features are retained, some on the coarsest level, "
                      "got {} with {} on the coarsest level",
                      fewFeatures, keyPoints.size(), coarsest));
}

} // namespace

int main()
{
    retainsRequestedFeatures();
    constexpr int levelFeatures = 400;
    // Fewer features than a cell holds.
    constexpr int fewFeatures = 6;

    cellsAreSquare(levelFeatures, width, height);
    cellsAreSquare(levelFeatures, height, width);
    cellsAreSquare(levelFeatures, 2 * width, height / 2);
    levelHasCell(fewFeatures, width, height);
    levelHasCell(1, height, width);
    levelHasCell(1, 2 * width, height / 2);
    cellsHoldFewFeatures(levelFeatures, width, height);
    cellsHoldFewFeatures(levelFeatures, height, width);
    cellsHoldFewFeatures(levelFeatures, 2 * width, height / 2);
    featurelessFrameIsComplete(cv::Mat::zeros(4, 1, CV_32F), "without distortion");
    featurelessFrameIsComplete(
        cv::Mat_<float>({4, 1}, {radialDistortion, 0, 0, 0}), "with distortion");

    return ORB_SLAM::tests::exitStatus();
}
