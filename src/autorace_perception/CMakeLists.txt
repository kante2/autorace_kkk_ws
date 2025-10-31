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
  pcl_ros            # ✅ LiDAR(PCL) 사용 시 ROS 브릿지
  pcl_conversions    # ✅ sensor_msgs <-> PCL 변환
)

find_package(OpenCV REQUIRED)
# ✅ PCL은 선택사항으로 처리(없으면 LiDAR 노드만 빌드 생략)
find_package(PCL QUIET)

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
    pcl_ros
    pcl_conversions
  DEPENDS
    OpenCV
)

# 전역 인클루드(헤더 포함 경로)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ------- 경로 변수 -------
set(LANE_DETECTION_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/lane_detection/src)
set(SENSOR_LIDAR_DIR       ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_lidar)

# ------- 라이브러리(lane_detection 코어) -------
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

# ------- 실행 파일: per_camera -------
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

# ------- 실행 파일: per_camera_rotary -------
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

# ------- 실행 파일: LiDAR(옵션) -------
if(PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  add_executable(lidar_obstacle_detect_node
    ${SENSOR_LIDAR_DIR}/lidar_obstacle_detect.cpp
  )
  add_dependencies(lidar_obstacle_detect_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(lidar_obstacle_detect_node
    PRIVATE
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
  )
else()
  message(WARNING "PCL not found. 'lidar_obstacle_detect_node' will not be built.")
endif()

# ------- 설치 -------
install(TARGETS
    lane_detection_core
    per_camera_node
    per_camera_rotary_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

if(PCL_FOUND)
  install(TARGETS
    lidar_obstacle_detect_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
