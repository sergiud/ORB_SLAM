// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "Sampling.h"

#include <cassert>
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

} // namespace ORB_SLAM
