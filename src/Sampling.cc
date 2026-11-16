// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "Sampling.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>

namespace ORB_SLAM
{

std::vector<std::size_t> DrawDistinctIndices(std::size_t count,
                                             std::size_t size,
                                             std::mt19937& generator)
{
    assert(count <= size);

    std::vector<std::size_t> available(size);
    std::iota(available.begin(), available.end(), std::size_t{0});

    std::vector<std::size_t> drawn;
    drawn.reserve(count);

    for (std::size_t i = 0; i < count; ++i) {
        std::uniform_int_distribution<std::size_t> dist(0, available.size() - 1);
        const std::size_t position = dist(generator);
        const std::size_t index = available[position];

        drawn.push_back(index);

        available[position] = available.back();
        available.pop_back();
    }

    return drawn;
}

int RansacIterations(double probability, double inlierRatio, int sampleSize,
                     int maxIterations)
{
    // With all correspondences inliers, any sample consists of inliers.
    if (!(inlierRatio < 1)) {
        return 1;
    }

    const double iterations =
        std::ceil(std::log1p(-probability) /
                  std::log1p(-std::pow(inlierRatio, sampleSize)));

    return static_cast<int>(
        std::clamp(iterations, 1.0, static_cast<double>(maxIterations)));
}

} // namespace ORB_SLAM
