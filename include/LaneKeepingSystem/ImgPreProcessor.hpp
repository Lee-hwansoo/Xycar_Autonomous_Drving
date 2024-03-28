/**
 * @file CameraUndistortion.hpp
 * @author Hwansoo Lee (lee_hwansoo@naver.com)
 * @brief Camera Calibration class header file
 * @version 1.1
 * @date 2024-04-26
 */
#ifndef IMG_PREPROCESSOR_HPP_
#define IMG_PREPROCESSOR_HPP_

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <vector>

namespace Xycar {

template <typename PREC>
class IMGPreProcessor {
public:
    using Ptr = std::unique_ptr<IMGPreProcessor<PREC>>; ///< Pointer type of this class

    /**
     * @brief Constructor for IMGPreProcessor
     *
     * @param config YAML configuration node
     * @param calibration YAML calibration node
     */
    IMGPreProcessor(const YAML::Node& config, const YAML::Node& calibration);

    /**
     * @brief Destructor for IMGPreProcessor
     */
    ~IMGPreProcessor();

    /**
     * @brief Preprocesses the image for lane detection
     *
     * @param image Input image for preprocessing
     * @param output Preprocessed image for lane detection
     */
    void preprocessImage(cv::Mat& image, cv::Mat& maskedRoiImage, cv::Mat& edgedRoiImage);

private:
    int32_t mImageWidth;            ///< The width of the image
    int32_t mImageHeight;           ///< The height of the image
    int32_t mROIStartHeight;        ///< The starting height of the region of interest (ROI)
    int32_t mROIHeight;             ///< The height of the region of interest (ROI)
    int32_t mCannyEdgeLowThreshold; ///< Low threshold for Canny edge detection
    int32_t mCannyEdgeHighThreshold;///< High threshold for Canny edge detection

    cv::Mat mCameraMatrix;          ///< Camera matrix for distortion correction
    cv::Mat mDistortionCoeffs;      ///< Distortion coefficients for distortion correction
};
} // namespace Xycar
#endif // IMG_PREPROCESSOR_HPP_
