#include <ros/ros.h>
#include <grid_map.h>
int main(int argc, char **argv){
    ros::init(argc, argv, "grid_map_node");
    ros::NodeHandle nh("~");
    GridMap grid_map;
    grid_map.initMap(nh);
    ros::spin();
}