#include "usb_cam.hpp"

namespace camera_driver
{
    UsbCam::UsbCam(UsbCamParam usb_params)
    {
        this->usb_cam_params_ = usb_params;
    }

    UsbCam::UsbCam()
    {

    }

    UsbCam::~UsbCam()
    {

    }

    void UsbCam::init()
    {
        // this->usb_cam_params_.camera_id = device;
        cap_.open(this->usb_cam_params_.camera_id);
        ROS_INFO("[USB CAMERA] ID:%d", this->usb_cam_params_.camera_id);
        if(cap_.isOpened())
        {
            this->is_open_ = true;
        }
    }

    // bool UsbCam::get_frame(cv::Mat &src)
    // {
    //     this->cap_ >> src;
    //     if(src.size().empty())
    //     {
    //         printf("grab image failed!");
    //         return false;
    //     }
    //     return true;
    // }   
} //namespace UsbCam