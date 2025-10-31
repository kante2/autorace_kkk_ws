cmake_minimum_required(VERSION 3.10)
project(autorace_perception)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
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
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    lane_detection_core
  CATKIN_DEPENDS
    geometry_msgs
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
  DEPENDS
    OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(LANE_DETECTION_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/lane_detection/src)

add_library(lane_detection_core
  ${LANE_DETECTION_SRC_DIR}/feature_extraction.cpp
  ${LANE_DETECTION_SRC_DIR}/preprocessing.cpp
  ${LANE_DETECTION_SRC_DIR}/sliding_window.cpp
)
add_dependencies(lane_detection_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(lane_detection_core
  PUBLIC
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
target_include_directories(lane_detection_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(per_camera_node
  ${LANE_DETECTION_SRC_DIR}/per_camera.cpp
)
add_dependencies(per_camera_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(per_camera_node
  PRIVATE
    lane_detection_core
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
target_include_directories(per_camera_node
  PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(per_camera_rotary_node
  ${LANE_DETECTION_SRC_DIR}/per_camera_rotary.cpp
)
add_dependencies(per_camera_rotary_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(per_camera_rotary_node
  PRIVATE
    lane_detection_core
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
target_include_directories(per_camera_rotary_node
  PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

install(TARGETS
    lane_detection_core
    per_camera_node
    per_camera_rotary_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
