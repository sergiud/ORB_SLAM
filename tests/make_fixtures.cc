// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Writes a short black video and a matching calibration into the working
// directory, enough for run_ORB_SLAM to process every frame without
// finding features.

#include <cstdlib>
#include <print>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace {

constexpr int width = 640;
constexpr int height = 480;
constexpr int frames = 3;
constexpr double framesPerSecond = 30;
constexpr double focalLength = 500;

} // namespace

int main()
{
    cv::VideoWriter writer{"black.avi",
                           cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                           framesPerSecond, cv::Size{width, height}};

    if (!writer.isOpened()) {
        std::println(stderr, "error: cannot write black.avi");
        return EXIT_FAILURE;
    }

    const cv::Mat black = cv::Mat::zeros(height, width, CV_8UC3);

    for (int i = 0; i < frames; ++i) {
        writer.write(black);
    }

    // clang-format off
    const cv::Matx33d K{
        focalLength, 0,           width / 2.0,
        0,           focalLength, height / 2.0,
        0,           0,           1};
    // clang-format on

    cv::FileStorage calibration{"calibration.yml", cv::FileStorage::WRITE};
    calibration << "camera_matrix" << cv::Mat{K};
    calibration << "distortion_coefficients" << cv::Mat1d::zeros(1, 4);

    return EXIT_SUCCESS;
}
