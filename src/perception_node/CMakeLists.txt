cmake_minimum_required(VERSION 3.0.2)
project(perception_node)

## C++ 표준 (원하면 14/17로 바꿔도 됨)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## catkin 패키지 의존성
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  image_transport
  visualization_msgs
  laser_geometry
  message_generation
  pcl_ros
  pcl_conversions
)

## OpenCV
find_package(OpenCV REQUIRED)

## PCL
find_package(PCL REQUIRED)

## catkin_package 설정
catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
    geometry_msgs
    cv_bridge
    image_transport
    visualization_msgs
    laser_geometry
    message_runtime
    pcl_ros
    pcl_conversions
  DEPENDS
    OpenCV
    PCL
)

## include 경로
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

########################################
##   노드 추가 편하게 쓰는 헬퍼 함수   ##
########################################
function(add_wego_node node_name)
  add_executable(${node_name}
    ${ARGN}
  )

  add_dependencies(${node_name}
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(${node_name}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
  )
endfunction()

#########################
## 실제 노드들 등록부  ##
#########################

# === lane ===
add_wego_node(lane_node
  src/camera/lane_mission/lane_node.cpp
)

# Gate mission lidar node
add_wego_node(gate_node
  src/lidar/gate_mission/gate_node.cpp
)

# Rotary mission lidar node
add_wego_node(rotary_node
  src/lidar/rotary_mission/rotary_node.cpp
)

# Parking mission lidar node
add_wego_node(parking_node
  src/lidar/parking_mission/parking_node.cpp
  src/lidar/parking_mission/lidar_preprocessing_parking.cpp
  src/lidar/parking_mission/parking_lot_detection.cpp
  src/lidar/parking_mission/goal_pose_pub.cpp
  src/lidar/parking_mission/parking_lot_common.hpp
)

# Labacorn perception node (DBSCAN markers + cloud)
add_wego_node(labacorn_node
  src/lidar/labacorn_mission/labacorn_node.cpp
  src/lidar/labacorn_mission/lidar_preprocessing_labacorn.cpp
)

## 나중에 새 노드 추가할 때는 밑에 한 줄만 추가:
## add_wego_node(새노드이름_node src/새파일이름.cpp)

#####################
## 설치 규칙 (옵션) ##
#####################
install(TARGETS
  lane_node
  gate_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
