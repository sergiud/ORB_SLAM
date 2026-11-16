// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests of drawing the minimal sets of RANSAC.

#include <algorithm>
#include <cstddef>
#include <format>
#include <random>
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

} // namespace

int main()
{
    drawsEveryIndexFirst(3, 4);
    drawsEveryIndexFirst(1, 2);
    drawsDistinctIndices(3, 4);
    drawsDistinctIndices(3, 3);
    drawsDistinctIndices(8, 20);

    return ORB_SLAM::tests::exitStatus();
}
