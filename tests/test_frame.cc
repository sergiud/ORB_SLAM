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

} // namespace

int main()
{
    featurelessFrameIsComplete(cv::Mat::zeros(4, 1, CV_32F), "without distortion");
    featurelessFrameIsComplete(
        cv::Mat_<float>({4, 1}, {radialDistortion, 0, 0, 0}), "with distortion");

    return ORB_SLAM::tests::exitStatus();
}
