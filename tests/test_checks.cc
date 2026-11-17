// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests of the preconditions that terminate the program with a failed check
// when they are violated.

#include <csignal>
#include <cstdlib>
#include <format>
#include <random>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

#include <ng-log/experimental/log_call.h>
#include <opencv2/core.hpp>

#include "Check.h"
#include "Converter.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "Sampling.h"
#include "Tracking.h"

namespace {

using ORB_SLAM::tests::check;

constexpr double framesPerSecond = 30;
constexpr int features = 1000;
constexpr float scaleFactor = 1.2F;
constexpr int levels = 8;
constexpr int fastThreshold = 20;

// Runs the statement in a child process and checks that it terminates with
// SIGABRT after writing a failed check that contains the expected text to
// the standard error.
template<class Statement>
void terminatesWithCheck(Statement statement, std::string_view expected)
{
    int pipe[2];

    if (::pipe(pipe) != 0) {
        check(false, std::format("{}: the test creates a pipe", expected));
        return;
    }

    const pid_t child = ::fork();

    if (child == 0) {
        ::dup2(pipe[1], STDERR_FILENO);
        ::close(pipe[0]);
        ::close(pipe[1]);
        statement();
        std::_Exit(EXIT_SUCCESS);
    }

    ::close(pipe[1]);

    std::string error;
    char buffer[4096];

    for (ssize_t n; (n = ::read(pipe[0], buffer, sizeof buffer)) > 0;) {
        error.append(buffer, static_cast<std::size_t>(n));
    }

    ::close(pipe[0]);

    int status = 0;
    ::waitpid(child, &status, 0);

    check(WIFSIGNALED(status) && WTERMSIG(status) == SIGABRT,
          std::format("{}: the program aborts, got status {:#x}", expected,
                      status));
    check(error.contains(std::format("Check failed: {}", expected)),
          std::format("the error reports \"Check failed: {}\", got {:?}",
                      expected, error));
}

// The extractor detects features in grayscale images only.
void extractorRejectsColorImage()
{
    constexpr int size = 64;

    terminatesWithCheck(
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
void drawingTooManyIndices()
{
    constexpr std::size_t count = 3;
    constexpr std::size_t size = 2;

    terminatesWithCheck(
        [count, size] {
            std::mt19937 generator;
            std::ignore =
                ORB_SLAM::DrawDistinctIndices(count, size, generator);
        },
        std::format("lhs <= rhs ({} vs. {})", count, size));
}

// A planar vector has two rows or two columns.
void convertingNonPlanarVector()
{
    terminatesWithCheck(
        [] {
            std::ignore = ORB_SLAM::Converter::toVector2d(
                cv::Mat::zeros(3, 1, CV_32F));
        },
        "a 2-vector is expected, got 3x1");
}

// The extractors score keypoints with Harris (0) or FAST (1) only.
void trackerRejectsScoreType()
{
    constexpr int score = 2;

    terminatesWithCheck(
        [] {
            ORB_SLAM::Tracking{nullptr, nullptr, nullptr, nullptr,
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
void rotationBinOutOfRange()
{
    // A rotation one bin past the last one, since the bins are 12 degrees
    // wide.
    constexpr float outOfRange = 372;

    if constexpr (nglog::experimental::IsDebugEnabled()) {
        terminatesWithCheck(
            [outOfRange] { std::ignore = BinningMatcher::RotationBin(outOfRange, 0); },
            "lhs in range");
    }
}

} // namespace

int main()
{
    extractorRejectsColorImage();
    drawingTooManyIndices();
    convertingNonPlanarVector();
    trackerRejectsScoreType();
    rotationBinOutOfRange();

    return ORB_SLAM::tests::exitStatus();
}
