#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

namespace Xycar {
class ImageToMP4 {
public:
    ImageToMP4() {
        image_subscriber = nh.subscribe("/usb_cam/image_raw", 1, &ImageToMP4::imageCallback, this);
        video_writer = nullptr;
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        try {
            cv::Mat frame;
            frame = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(msg->data.data()));

            if (video_writer == nullptr) {
                int width = frame.cols;
                int height = frame.rows;
                std::string out_filename = "output_video.mp4";
                video_writer = std::make_unique<cv::VideoWriter>(out_filename, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30.0, cv::Size(width, height));
            }

            video_writer->write(frame);
            ROS_INFO("Frame written to video.");
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM(e.what());
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber image_subscriber;
    std::unique_ptr<cv::VideoWriter> video_writer;
};
} // namespace Xycar

// Usage:
// int main(int argc, char** argv) {
//     ros::init(argc, argv, "image_to_mp4");
//     Xycar::ImageToMP4 image_to_mp4;
//     ros::spin();
//     return 0;
// }
