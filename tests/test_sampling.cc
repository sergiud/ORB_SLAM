// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests of drawing the minimal sets of RANSAC.

#include <algorithm>
#include <cstddef>
#include <format>
#include <random>
#include <utility>
#include <vector>

#include "Check.h"
#include "Sampling.h"

namespace {

using ORB_SLAM::tests::check;

// Enough draws that every way of drawing from a small pool occurs.
constexpr int draws = 1000;

// Every draw holds count distinct indices below size.
void drawsDistinctIndices(std::size_t count, std::size_t size)
{
    std::mt19937 generator;

    for (int i = 0; i < draws; ++i) {
        std::vector<std::size_t> sample =
            ORB_SLAM::DrawDistinctIndices(count, size, generator);

        check(sample.size() == count,
              std::format("{} of {} indices are drawn, got {}", count, size,
                          sample.size()));

        const bool inRange = std::ranges::all_of(
            sample, [size](std::size_t index) { return index < size; });
        std::ranges::sort(sample);
        const bool distinct =
            std::ranges::adjacent_find(sample) == sample.end();

        check(inRange && distinct,
              std::format("draw {} of {} of {} indices holds distinct indices "
                          "below {}, got {}",
                          i, count, size, size, sample));

        if (!(inRange && distinct)) {
            return;
        }
    }
}

// Every index comes first in some draw, so that no index is out of reach.
void drawsEveryIndexFirst(std::size_t count, std::size_t size)
{
    std::mt19937 generator;
    std::vector<int> first(size);

    for (int i = 0; i < draws; ++i) {
        ++first[ORB_SLAM::DrawDistinctIndices(count, size, generator).front()];
    }

    check(std::ranges::find(first, 0) == first.end(),
          std::format("every one of {} indices comes first in {} draws of {}, "
                      "got {} times each",
                      size, draws, count, first));
}

// The iterations draw a sample of inliers with the given probability, using
// the size of the sample.
void ransacIterationsFollowSampleSize()
{
    constexpr double probability = 0.99;
    constexpr double inlierRatio = 0.5;
    constexpr int maxIterations = 300;

    // ceil(log(0.01) / log(1 - 0.5^k)) for k = 3 and k = 4.
    constexpr int threePoints = 35;
    constexpr int fourPoints = 72;

    for (const auto [sampleSize, expected] :
         {std::pair{3, threePoints}, std::pair{4, fourPoints}}) {
        const int iterations = ORB_SLAM::RansacIterations(
            probability, inlierRatio, sampleSize, maxIterations);

        check(iterations == expected,
              std::format("samples of {} need {} iterations, got {}",
                          sampleSize, expected, iterations));
    }
}

// With more inliers required than correspondences exist, the inlier ratio
// exceeds 1 and a single iteration is enough to find out.
void ransacIterationsForAllInliers()
{
    constexpr double probability = 0.99;
    constexpr int sampleSize = 4;
    constexpr int maxIterations = 300;

    for (const double inlierRatio : {1.0, 1.5}) {
        const int iterations = ORB_SLAM::RansacIterations(
            probability, inlierRatio, sampleSize, maxIterations);

        check(iterations == 1,
              std::format("an inlier ratio of {} needs 1 iteration, got {}",
                          inlierRatio, iterations));
    }
}

// The number of iterations stays between 1 and the given maximum.
void ransacIterationsWithinBounds()
{
    constexpr double inlierRatio = 0.5;
    constexpr int sampleSize = 4;
    constexpr int maxIterations = 50;

    const int capped =
        ORB_SLAM::RansacIterations(0.99, inlierRatio, sampleSize, maxIterations);
    const int least =
        ORB_SLAM::RansacIterations(0, inlierRatio, sampleSize, maxIterations);

    check(capped == maxIterations,
          std::format("at most {} iterations run, got {}", maxIterations,
                      capped));
    check(least == 1, std::format("at least 1 iteration runs, got {}", least));
}

} // namespace

int main()
{
    ransacIterationsWithinBounds();
    ransacIterationsFollowSampleSize();
    ransacIterationsForAllInliers();
    drawsEveryIndexFirst(3, 4);
    drawsEveryIndexFirst(1, 2);
    drawsDistinctIndices(3, 4);
    drawsDistinctIndices(3, 3);
    drawsDistinctIndices(8, 20);

    return ORB_SLAM::tests::exitStatus();
}
