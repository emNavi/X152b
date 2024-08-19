#ifndef USB_CAM_HPP_
#define USB_CAM_HPP_

// ros
#include <ros/ros.h>

// c++
#include <iostream>

// opencv
#include <opencv2/opencv.hpp>

namespace camera_driver
{
    struct UsbCamParam
    {
        std::string frame_id;
        int camera_id;
        int image_width;
        int image_height;
        int fps;
        bool show_img_;
        bool debug_;
        bool using_bag_;
        std::string camera_param_path_;
        bool save_video_;
        std::string save_path_;
        bool using_video_;
        std::string video_path_;
    };

    class UsbCam
    {
    public:
        UsbCam();
        UsbCam(UsbCamParam usb_params);
        ~UsbCam();

        void init();

        // void get_params();
        // int start_device(int serial_num);
        // bool get_frame(cv::Mat &src);

    private:
        // std::string device_path;
        UsbCamParam usb_cam_params_;
        // rclcpp::Logger logger_;

    public:
        cv::VideoCapture cap_;
        bool is_open_;
        cv::Mat src_;
    };   // usb_cam
}   // namespace camera_driver

#endif