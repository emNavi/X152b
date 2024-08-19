#ifndef USB_CAM_NODE_HPP_
#define USB_CAM_NODE_HPP_

#include <memory>
#include <string>

// opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

// ros
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include <string>

#include "usb_cam.hpp"
#include "camera_driver/UsbCameraConfig.h"
#include "dynamic_reconfigure/server.h"

namespace camera_driver
{
    // class UsbCamNode : public rclcpp::Node
    class UsbCamNode
    {
        // TODO(Derkai):考虑后面留作相机动态调参接口
        // typedef ros::ParameterEventHandler ParamSubscriber;
        // typedef rclcpp::ParameterCallbackHandle ParamCbHandle;
        // typedef rclcpp::ParameterEventHandler::ParameterCallbackType ParamCallbackType;

    public:
        UsbCamNode();
        // UsbCamNode(const rclcpp::NodeOptions& option = rclcpp::NodeOptions());
        ~UsbCamNode();

    private:
        ros::NodeHandle n;
        dynamic_reconfigure::Server<camera_driver::UsbCameraConfig> usb_camera_config_server;
        dynamic_reconfigure::Server<camera_driver::UsbCameraConfig>::CallbackType usb_camera_config_callback_type;

        cv::Mat raw_frame_;
        cv::Mat frame_;
        cv::Mat filpped_frame_;
        sensor_msgs::Image::Ptr msg_;
        cv::VideoCapture cap_;
        bool is_filpped_;
        // rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

        // std::unique_ptr<usb_cam> usb_cam_;

        // ros::TimerBase::SharedPtr img_pub_timer_;
        ros::WallTimer img_pub_timer_;
        // ros::Publisher<sensor_msgs::Image>::SharedPtr image_msg_pub_;
        ros::Publisher image_msg_pub_;

        ros::Time last_time_;
        // std::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;
        // image_transport::CameraPublisher camera_info_pub_;
        // std::shared_ptr<sensor_msgs::msg::Image> image_msg_;

    public:
        void imageCallback();
        std::shared_ptr<sensor_msgs::Image> convertFrameToMessage(cv::Mat& frame);
        void initImageUndistort(std::string camera_param_path);
        void usbCameraParamCallback(camera_driver::UsbCameraConfig& config, uint32_t level);

    private:
        cv::VideoWriter writer_;
        int frame_cnt_;
        // std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
        // std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader_;
        sensor_msgs::Image image_msg_;

        cv::Mat camera_matrix           = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat new_camera_matrix       = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);
        cv::Mat mapx, mapy;
        double bias_x1, bias_x2, bias_y1, bias_y2;
        UsbCamParam usb_cam_params_;
        std::unique_ptr<UsbCam> usb_cam_;
        std::unique_ptr<UsbCam> initUsbCamParam();

    protected:
        // std::shared_ptr<ParamSubscriber> param_subscriber_;
        // std::shared_ptr<ParamCbHandle> param_cb_handle_;

        // params callback.
        std::map<std::string, int> param_map_;
        bool setParam();
        bool getParam();
        // bool setParam(ros::param param);
        // rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::param>& params);
        // OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    };   // UsbCamNode
}   // namespace camera_driver

#endif