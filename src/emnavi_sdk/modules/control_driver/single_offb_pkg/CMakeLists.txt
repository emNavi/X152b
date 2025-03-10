cmake_minimum_required(VERSION 3.0.2)
project(single_offb_pkg)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  quadrotor_msgs
)

catkin_package(
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(uav_ctrl src/uav_ctrl.cpp src/qr_controller.cpp)
target_link_libraries(uav_ctrl
  ${catkin_LIBRARIES}
)

add_executable(mission_ctrl src/mission_ctrl.cpp)
target_link_libraries(mission_ctrl
  ${catkin_LIBRARIES}
)
