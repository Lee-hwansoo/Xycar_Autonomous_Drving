/**
 * @file ImgPreProcessor.cpp
 * @author Hwansoo Lee (lrrghdrh@naver.com)
 * @brief camera calibration class source file
 * @version 1.1
 * @date 2024-04-26
 */

#include "LaneKeepingSystem/ImgPreProcessor.hpp"

namespace Xycar {

template <typename PREC>
IMGPreProcessor<PREC>::IMGPreProcessor(const YAML::Node& config, const YAML::Node& calibration) {
    mImageWidth = config["IMAGE"]["WIDTH"].as<int32_t>();
    mImageHeight = config["IMAGE"]["HEIGHT"].as<int32_t>();
    mROIStartHeight = config["IMAGE"]["ROI_START_HEIGHT"].as<int32_t>();
    mROIHeight = config["IMAGE"]["ROI_HEIGHT"].as<int32_t>();
    mCannyEdgeLowThreshold = config["CANNY"]["LOW_THRESHOLD"].as<int32_t>();
    mCannyEdgeHighThreshold = config["CANNY"]["HIGH_THRESHOLD"].as<int32_t>();

    // Load camera calibration parameters
    std::vector<PREC> camera_matrix_data = calibration["camera_matrix"]["data"].as<std::vector<PREC>>();
    std::vector<PREC> dist_coeffs_data = calibration["distortion_coefficients"]["data"].as<std::vector<PREC>>();

    // Creating cv::Mat from std::vector and reshaping it
    mCameraMatrix = cv::Mat(camera_matrix_data, true).reshape(0, 3);
    mDistortionCoeffs = cv::Mat(dist_coeffs_data, true);

    // Determine the data type based on PREC and convert cv::Mat accordingly
    int cvType = (typeid(PREC) == typeid(float)) ? CV_32F : CV_64F;
    mCameraMatrix.convertTo(mCameraMatrix, cvType);
    mDistortionCoeffs.convertTo(mDistortionCoeffs, cvType);
}

template <typename PREC>
IMGPreProcessor<PREC>::~IMGPreProcessor() {}

template <typename PREC>
void IMGPreProcessor<PREC>::preprocessImage(cv::Mat& image, cv::Mat& maskedRoiImage, cv::Mat& edgedRoiImage) {
    // Undistort the image
    cv::Mat undistortedImage;
    cv::undistort(image, undistortedImage, mCameraMatrix, mDistortionCoeffs);

    // Apply Gaussian blur
    cv::Mat blurredImage;
    cv::GaussianBlur(undistortedImage, blurredImage, cv::Size(5, 5), 0);

    // Convert image to HLS color space
    cv::Mat hlsImage;
    cv::cvtColor(blurredImage, hlsImage, cv::COLOR_BGR2HLS);

    // Define lower and upper bounds for black color (in HLS space)
    cv::Scalar lowerBlack(0, 0, 0);
    cv::Scalar upperBlack(220, 220, 55);

    // Create mask for black color
    cv::Mat blackMask;
    cv::inRange(hlsImage, lowerBlack, upperBlack, blackMask);

    // Apply Canny edge detection
    cv::Mat edgesImage;
    cv::Canny(blackMask, edgesImage, mCannyEdgeLowThreshold, mCannyEdgeHighThreshold);

    // Apply ROI
    cv::Mat roiImage1 = blackMask(cv::Rect(0, mROIStartHeight, mImageWidth, mROIHeight));
    cv::Mat roiImage2 = edgesImage(cv::Rect(0, mROIStartHeight, mImageWidth, mROIHeight));

    maskedRoiImage = roiImage1.clone(); // Cloning the ROI image for further processing -> 정지선 검출
    edgedRoiImage = roiImage2.clone(); // Cloning the ROI image for further processing -> 차선 검출
}

// Explicit instantiation of template class for specific types
template class IMGPreProcessor<float>;
template class IMGPreProcessor<double>;
} // namespace Xycar
