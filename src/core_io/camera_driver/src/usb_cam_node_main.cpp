#include "usb_cam_node.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "usb_cam_driver_node");
    ros::NodeHandle n;

    //定时器1每1ms执行一次
    camera_driver::UsbCamNode();
    ros::spin();
    return 0;
}
