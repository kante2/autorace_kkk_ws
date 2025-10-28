cmake_minimum_required(VERSION 3.0.2)
project(autorace_perception)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  morai_msgs
  nav_msgs
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  usb_cam
  image_proc
  image_view
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autorace_perception
#  CATKIN_DEPENDS geometry_msgs message_generation morai_msgs nav_msgs roscpp std_msgs tf2 tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(camera_calibration src/camera_calibration.cpp)
#target_compile_features(camera_calibration PRIVATE cxx_std_11)
#target_link_libraries(camera_calibration ${catkin_LIBRARIES})
