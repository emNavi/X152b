cmake_minimum_required(VERSION 2.8.3)
project(msg_socket_bridge)
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g")
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)
find_package(PCL 1.7 REQUIRED)

add_message_files(
  FILES
  Bspline.msg
  MINCOTraj.msg
  PositionTarget.msg
  )
  
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
 CATKIN_DEPENDS message_runtime
)
include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(socket_bridge_node
  src/bridge_node.cpp 
  )
add_dependencies(socket_bridge_node msg_socket_bridge_generate_messages_cpp)
target_link_libraries(socket_bridge_node 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )