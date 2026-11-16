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

} // namespace ORB_SLAM

#endif // ORB_SLAM_SAMPLING_H
