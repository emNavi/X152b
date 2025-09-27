cmake_minimum_required(VERSION 2.8.3)
project(local_ocmap_gen)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
  tf_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES local_ocmap_gen
 CATKIN_DEPENDS roscpp std_msgs
 DEPENDS OpenCV
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

# # 动态障碍块场景生成
# add_executable(obj_generator
#     src/obj_generator.cpp 
# )
# target_link_libraries(obj_generator 
#     ${catkin_LIBRARIES}
#     )

# 建图
add_executable(${PROJECT_NAME}_node 
    src/grid_map_node.cpp 
    src/grid_map.cpp src/raycast.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(${PROJECT_NAME}_tf_node src/baselink_world_tf_pub.cpp)

target_link_libraries(${PROJECT_NAME}_tf_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${OpenCV_LIBS}
)