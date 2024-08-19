cmake_minimum_required(VERSION 3.0.2)
project(serial_driver)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_minimal_nodes
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_driver_node src/serial_node.cpp)

target_link_libraries(serial_driver_node
  ${catkin_LIBRARIES}
)
