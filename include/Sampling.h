// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef ORB_SLAM_SAMPLING_H
#define ORB_SLAM_SAMPLING_H

#include <cstddef>
#include <random>
#include <vector>

#include "ORB_SLAM_export.h"

namespace ORB_SLAM
{

/**
 * Draws distinct indices uniformly at random, such as a minimal set of a
 * RANSAC iteration.
 *
 * @param count Number of indices to draw, at most @p size.
 * @param size Number of indices to draw from, which are 0 to @p size - 1.
 * @param generator Source of the random numbers.
 * @return The @p count distinct indices in the order they were drawn.
 */
ORB_SLAM_EXPORT std::vector<std::size_t> DrawDistinctIndices(
    std::size_t count, std::size_t size, std::mt19937& generator);

/**
 * Returns the number of RANSAC iterations that draw at least one sample of
 * inliers with the given probability.
 *
 * A sample of @f$k@f$ correspondences consists of inliers only with
 * probability @f$\varepsilon^k@f$ for the inlier ratio @f$\varepsilon@f$, so
 * that @f$n@f$ iterations succeed with probability @f$p@f$ for
 * @f[
 *     n = \left\lceil \frac{\log(1 - p)}{\log(1 - \varepsilon^k)} \right\rceil .
 * @f]
 *
 * @param probability Probability @f$p@f$ that one sample consists of inliers.
 * @param inlierRatio Expected ratio @f$\varepsilon@f$ of inliers among the
 *        correspondences.
 * @param sampleSize Number @f$k@f$ of correspondences in a sample.
 * @param maxIterations Largest number of iterations to return.
 * @return The number of iterations, between 1 and @p maxIterations.
 */
ORB_SLAM_EXPORT int RansacIterations(double probability, double inlierRatio,
                                     int sampleSize, int maxIterations);

} // namespace ORB_SLAM

#endif // ORB_SLAM_SAMPLING_H
