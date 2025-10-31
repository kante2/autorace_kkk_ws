cmake_minimum_required(VERSION 3.0.2)
project(autorace_perception)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  morai_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cv_bridge
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  usb_cam
  image_proc
  image_view
  sensor_msgs
)

find_package(OpenCV REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autorace_perception
#  CATKIN_DEPENDS geometry_msgs message_generation morai_msgs nav_msgs roscpp std_msgs tf2 tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

#[[
#add_executable(camera_calibration src/camera_calibration.cpp)
#target_compile_features(camera_calibration PRIVATE cxx_std_11)
#target_link_libraries(camera_calibration ${catkin_LIBRARIES})
]]

add_library(lane_detection_core
  src/sensor_camera/lane_detection/src/feature_extraction.cpp
  src/sensor_camera/lane_detection/src/preprocessing.cpp
  src/sensor_camera/lane_detection/src/sliding_window.cpp
)
add_dependencies(lane_detection_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lane_detection_core
  PRIVATE
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
target_include_directories(lane_detection_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/lane_detection/include>
    $<INSTALL_INTERFACE:include>
)
add_dependencies(lane_detection_core ${catkin_EXPORTED_TARGETS})

add_executable(per_camera_node
  src/sensor_camera/lane_detection/src/per_camera.cpp
)
add_dependencies(per_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(per_camera_node
  PRIVATE
    lane_detection_core
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_dependencies(per_camera_node ${catkin_EXPORTED_TARGETS})

add_executable(per_camera_rotary_node
  src/sensor_camera/lane_detection/src/per_camera_rotary.cpp
)
add_dependencies(per_camera_rotary_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(per_camera_rotary_node
  PRIVATE
    lane_detection_core
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_dependencies(per_camera_rotary_node ${catkin_EXPORTED_TARGETS})

install(TARGETS
    lane_detection_core
    per_camera_node
    per_camera_rotary_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY src/lane_detection/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
