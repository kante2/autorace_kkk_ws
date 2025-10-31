cmake_minimum_required(VERSION 3.10)
project(lane_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lane_detection_core
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(lane_detection_core
  src/preprocessing.cpp
  src/feature_extraction.cpp
  src/sliding_window.cpp
)
target_link_libraries(lane_detection_core
  PUBLIC
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
target_include_directories(lane_detection_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
add_dependencies(lane_detection_core ${catkin_EXPORTED_TARGETS})

add_executable(per_camera_node
  src/per_camera.cpp
)
target_link_libraries(per_camera_node
  PRIVATE
    lane_detection_core
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_dependencies(per_camera_node ${catkin_EXPORTED_TARGETS})

add_executable(per_camera_rotary_node
  src/per_camera_rotary.cpp
)
target_link_libraries(per_camera_rotary_node
  PRIVATE
    lane_detection_core
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_dependencies(per_camera_rotary_node ${catkin_EXPORTED_TARGETS})

install(TARGETS lane_detection_core per_camera_node per_camera_rotary_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
