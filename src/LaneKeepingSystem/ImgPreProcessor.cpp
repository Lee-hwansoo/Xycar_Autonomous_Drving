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
void IMGPreProcessor<PREC>::preprocessImage(const cv::Mat& image, cv::Mat& blurredRoiImage ,cv::Mat& edgedRoiImage) {
    // Convert image to HLS color space
    cv::Mat hlsImage;
    cv::cvtColor(image, hlsImage, cv::COLOR_BGR2HLS);

    // Define lower and upper bounds for black color (in HLS space)
    cv::Scalar lowerBlack(0, 0, 0);
    cv::Scalar upperBlack(220, 220, 55);

    // Create mask for black color
    cv::Mat blackMask;
    cv::inRange(hlsImage, lowerBlack, upperBlack, blackMask);

    // Undistort the image
    cv::Mat undistortedImage;
    cv::undistort(blackMask, undistortedImage, mCameraMatrix, mDistortionCoeffs);

    // Convert undistorted image to grayscale
    cv::Mat grayImage;
    cv::cvtColor(undistortedImage, grayImage, cv::COLOR_BGR2GRAY);

    // Apply Gaussian blur
    cv::Mat blurredImage;
    cv::GaussianBlur(grayImage, blurredImage, cv::Size(7, 7), 0);

    // Apply Canny edge detection
    cv::Mat edgesImage;
    cv::Canny(blurredImage, edgesImage, mCannyEdgeLowThreshold, mCannyEdgeHighThreshold);

    // Apply ROI
    cv::Mat roiImage1 = blurredImage(cv::Rect(0, mROIStartHeight, mImageWidth, mROIHeight));
    cv::Mat roiImage2 = edgesImage(cv::Rect(0, mROIStartHeight, mImageWidth, mROIHeight));

    blurredRoiImage = roiImage1.clone(); // Cloning the ROI image for further processing -> 정지선 검출
    edgedRoiImage = roiImage2.clone(); // Cloning the ROI image for further processing -> 차선 검출
}

// Explicit instantiation of template class for specific types
template class IMGPreProcessor<float>;
template class IMGPreProcessor<double>;
} // namespace Xycar

// Usage:
// #include "LaneKeepingSystem/ImgPreProcessor.hpp"
// #include <opencv2/opencv.hpp>
// #include <yaml-cpp/yaml.h>
// #include <memory>

// int main() {
//     // 비디오 파일 열기
//     cv::VideoCapture video("input_video.mp4");

//     // 비디오 파일이 정상적으로 열렸는지 확인
//     if (!video.isOpened()) {
//         std::cerr << "Error: Failed to open video file!" << std::endl;
//         return -1;
//     }

//     // YAML 파일 로드
//     YAML::Node config = YAML::LoadFile("config.yaml");
//     YAML::Node calibration = YAML::LoadFile("calibration.yaml");

//     // IMGPreProcessor 객체 생성
//     std::unique_ptr<Xycar::IMGPreProcessor<float>> preprocessor =
//         std::make_unique<Xycar::IMGPreProcessor<float>>(config, calibration);

//     // 이미지를 담을 윈도우 생성
//     cv::namedWindow("Concatenated Image", cv::WINDOW_NORMAL);

//     // 프레임 단위로 영상 처리
//     cv::Mat src;
//     while (video.read(src)) {
//         // 전처리된 이미지 생성
//         cv::Mat frame;
//         cv::cvtColor(src, frame, cv::COLOR_RGB2BGR);
//         cv::Mat blurredRoiFrame;
//         cv::Mat edgedRoiFrame;
//         preprocessor->preprocessImage(frame, blurredRoiFrame, edgedRoiFrame);

//         // 이미지 연결(concatenate)
//         cv::Mat concatenatedImage;
//         cv::hconcat(src, edgedRoiFrame, concatenatedImage);

//         // 연결된 이미지 표시
//         cv::imshow("Concatenated Image", concatenatedImage);

//         // 키 입력을 기다림 (ESC 키를 누르면 종료)
//         if (cv::waitKey(25) == 27)
//             break;
//     }

//     // 비디오 파일 닫기 및 윈도우 제거
//     video.release();
//     cv::destroyAllWindows();

//     return 0;
// }
