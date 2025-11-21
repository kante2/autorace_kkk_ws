cmake_minimum_required(VERSION 3.0.2)
project(jiwon_mission)

## catkin 패키지들
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  laser_geometry
  visualization_msgs
  geometry_msgs
)

## OpenCV (bev_binary, stopline_ratio 에서 사용)
find_package(OpenCV REQUIRED)

## PCL (gate 노드에서 사용)
find_package(PCL REQUIRED COMPONENTS common kdtree)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
    laser_geometry
    visualization_msgs
    geometry_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})


# --- gate (mission7 게이트 노드) ---
add_executable(gate src/gate.cpp)
target_link_libraries(gate
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
