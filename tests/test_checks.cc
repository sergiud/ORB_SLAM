// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests of the preconditions that terminate the program with a failed check
// when they are violated. The program runs the case it is given, and
// ExpectCheckFailure.cmake judges how the program terminates.

#include <array>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <print>
#include <random>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#ifdef _MSC_VER
#include <stdlib.h>
#endif // _MSC_VER

#include <ng-log/experimental/log_call.h>
#include <opencv2/core.hpp>

#include "Converter.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "Sampling.h"
#include "Tracking.h"

namespace {

constexpr double framesPerSecond = 30;
constexpr int features = 1000;
constexpr float scaleFactor = 1.2F;
constexpr int levels = 8;
constexpr int fastThreshold = 20;

// Writes the text that the failed check contains to the standard output and
// runs the statement, which terminates the program unless the check is
// missing.
template<class Statement>
int terminatesWithCheck(Statement statement, std::string_view expected)
{
    std::println("{}", expected);
    // The output must outlive the termination, which flushes no streams.
    std::fflush(stdout);
    statement();
    return EXIT_SUCCESS;
}

// The extractor detects features in grayscale images only.
int extractorRejectsColorImage()
{
    constexpr int size = 64;

    return terminatesWithCheck(
        [] {
            ORB_SLAM::ORBextractor extractor{
                features, scaleFactor, levels,
                ORB_SLAM::ORBextractor::FAST_SCORE, fastThreshold};
            const cv::Mat color = cv::Mat::zeros(size, size, CV_8UC3);
            std::vector<cv::KeyPoint> keyPoints;
            cv::Mat descriptors;
            extractor(color, cv::Mat{}, keyPoints, descriptors);
        },
        std::format("lhs == rhs ({} vs. {}) image type", CV_8UC3, CV_8UC1));
}

// More distinct indices than available cannot be drawn.
int drawingTooManyIndices()
{
    constexpr std::size_t count = 3;
    constexpr std::size_t size = 2;

    return terminatesWithCheck(
        [count, size] {
            std::mt19937 generator;
            std::ignore =
                ORB_SLAM::DrawDistinctIndices(count, size, generator);
        },
        std::format("lhs <= rhs ({} vs. {})", count, size));
}

// A planar vector has two rows or two columns.
int convertingNonPlanarVector()
{
    return terminatesWithCheck(
        [] {
            std::ignore = ORB_SLAM::Converter::toVector2d(
                cv::Mat::zeros(3, 1, CV_32F));
        },
        "a 2-vector is expected, got 3x1");
}

// The extractors score keypoints with Harris (0) or FAST (1) only.
int trackerRejectsScoreType()
{
    constexpr int score = 2;

    return terminatesWithCheck(
        [] {
            ORB_SLAM::Tracking{nullptr, nullptr, nullptr,
                               cv::Mat1f::eye(3, 3), cv::Mat{},
                               framesPerSecond, features, scaleFactor,
                               levels, fastThreshold, score,
                               /*useMotionModel=*/true};
        },
        std::format("the score type must be 0 (Harris) or 1 (FAST), got {}",
                    score));
}

// Exposes how the matcher assigns rotations to histogram bins.
class BinningMatcher : public ORB_SLAM::ORBmatcher
{
public:
    using ORB_SLAM::ORBmatcher::RotationBin;
};

// A rotation between angles in degrees falls into one of the bins, which
// debug builds check.
int rotationBinOutOfRange()
{
    // A rotation one bin past the last one, since the bins are 12 degrees
    // wide.
    constexpr float outOfRange = 372;

    if constexpr (!nglog::experimental::IsDebugEnabled()) {
        std::println(stderr, "skipped: the checks of debug builds are disabled");
        return EXIT_SUCCESS;
    }

    return terminatesWithCheck(
        [outOfRange] { std::ignore = BinningMatcher::RotationBin(outOfRange, 0); },
        "lhs in range");
}

} // namespace

int main(int argc, char** argv)
{
#ifdef _MSC_VER
    // Debug builds of MSVC report an abort in a dialog that waits for the user.
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif // _MSC_VER

    constexpr std::array cases{
        std::pair{std::string_view{"extractor-rejects-color-image"},
                  &extractorRejectsColorImage},
        std::pair{std::string_view{"drawing-too-many-indices"},
                  &drawingTooManyIndices},
        std::pair{std::string_view{"converting-non-planar-vector"},
                  &convertingNonPlanarVector},
        std::pair{std::string_view{"tracker-rejects-score-type"},
                  &trackerRejectsScoreType},
        std::pair{std::string_view{"rotation-bin-out-of-range"},
                  &rotationBinOutOfRange},
    };

    if (argc == 2) {
        for (const auto& [name, run] : cases) {
            if (name == argv[1]) {
                return run();
            }
        }
    }

    std::println(stderr, "usage: test_checks <case>, got {} arguments",
                 argc - 1);
    return EXIT_FAILURE;
}
