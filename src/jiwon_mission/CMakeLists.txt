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


add_executable(stopline_ratio
  src/stopline_ratio.cpp
)
target_link_libraries(stopline_ratio
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(stopline_ratio_imagetest
  src/stopline_ratio_imagetest.cpp
)
target_link_libraries(stopline_ratio_imagetest
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(leftright
  src/leftright.cpp
)
target_link_libraries(leftright
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(leftright_imagetest
  src/leftright_imagetest.cpp
)
target_link_libraries(leftright_imagetest
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)