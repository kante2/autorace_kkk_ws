cmake_minimum_required(VERSION 3.0.2)
project(stopline)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stopline
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(bev_binary src/bev_binary.cpp)
target_compile_features(bev_binary PRIVATE cxx_std_11)
target_link_libraries(bev_binary ${catkin_LIBRARIES})

add_executable(stopline_ratio src/stopline_ratio.cpp)
target_compile_features(stopline_ratio PRIVATE cxx_std_11)
target_link_libraries(stopline_ratio ${catkin_LIBRARIES})