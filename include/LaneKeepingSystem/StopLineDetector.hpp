/**
 * @file StopLineDetector.hpp
 * @author Hwansoo Lee (lee_hwansoo@naver.com)
 * @brief StopLineDetector class header file
 * @version 1.1
 * @date 2024-04-26
 */
#ifndef STOP_LINE_DETECTOR_HPP_
#define STOP_LINE_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <memory>

namespace Xycar {

template <typename PREC>
class StopLineDetector {
public:
    using Ptr = std::unique_ptr<StopLineDetector<PREC>>; ///< Pointer type of this class

    StopLineDetector(const YAML::Node& config);

    ~StopLineDetector();

    bool detect(const cv::Mat& maskedRoiImage);

private:
    cv::Rect ROI;                    ///< The region of interest for detection
    int32_t mImageWidth;            ///< The width of the image
    int32_t mImageHeight;           ///< The height of the image
    int32_t mROIStartHeight;        ///< The starting height of the region of interest (ROI)
    int32_t mROIHeight;             ///< The height of the region of interest (ROI)
    int32_t STARTHEIGHT;            ///< The starting height of the region of interest (ROI)
    int32_t HEIGHT;                 ///< The height of the region of interest (ROI)
    int32_t STARTWIDTH;             ///< The starting width of the region of interest (ROI)
    int32_t WIDTH;                  ///< The width of the region of interest (ROI)

    int32_t nonZero;                ///< The number of non-zero pixels in the region of interest (ROI)
    int32_t THRESHOLD;              ///< The threshold for detecting the stop line
};
} // namespace Xycar
#endif // STOP_LINE_DETECTOR_HPP_
