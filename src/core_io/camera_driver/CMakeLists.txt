cmake_minimum_required(VERSION 3.16.0)
project(camera_driver)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rosbag
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/UsbCamera.cfg
)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES camera_driver
 CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp  sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros dynamic_reconfigure
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include/camera
)

add_library(${PROJECT_NAME}
  src/usb_cam.cpp
  src/usb_cam_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
)

add_executable(usb_cam_driver_node src/usb_cam_node_main.cpp src/usb_cam.cpp src/usb_cam_node.cpp)
target_link_libraries(usb_cam_driver_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
)
