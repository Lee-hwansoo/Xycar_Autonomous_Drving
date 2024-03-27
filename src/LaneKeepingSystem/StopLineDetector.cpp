/**
 * @file StopLineDetector.cpp
 * @author Hwansoo Lee (lrrghdrh@naver.com)
 * @brief StopLineDetector class source file
 * @version 1.1
 * @date 2024-04-26
 */

#include "LaneKeepingSystem/StopLineDetector.hpp"

namespace Xycar {

template <typename PREC>
StopLineDetector<PREC>::StopLineDetector(const YAML::Node& config) {
    mImageWidth = config["IMAGE"]["WIDTH"].as<int32_t>();
    mImageHeight = config["IMAGE"]["HEIGHT"].as<int32_t>();
    mROIStartHeight = config["IMAGE"]["ROI_START_HEIGHT"].as<int32_t>();
    mROIHeight = config["IMAGE"]["ROI_HEIGHT"].as<int32_t>();

    STARTHEIGHT = 20;
    HEIGHT = 20;
    STARTWIDTH = 100;
    WIDTH = mImageWidth - (2 * STARTWIDTH);
    ROI = cv::Rect(STARTWIDTH, STARTHEIGHT, WIDTH, HEIGHT);   // cv::Rect(x, y, width, height)
    THRESHOLD = WIDTH * HEIGHT * 0.7;
}

template <typename PREC>
StopLineDetector<PREC>::~StopLineDetector() {}

template <typename PREC>
bool StopLineDetector<PREC>::detect(cv::Mat& blurredRoiImage) {
    cv::Mat stopLine = blurredRoiImage(ROI);
    nonZero = cv::countNonZero(stopLine);
    return (nonZero> THRESHOLD) ? true : false;
}

// Explicit instantiation of template class for specific types
template class StopLineDetector<float>;
template class StopLineDetector<double>;
} // namespace Xycar
