cmake_minimum_required(VERSION 3.0.2)
project(autorace_control)

## -----------------------------------------------------------------------------
## Catkin deps
## -----------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  visualization_msgs
  morai_msgs
  rosbag
  laser_geometry
)

## -----------------------------------------------------------------------------
## catkin_package
##  - INCLUDE_DIRS 에 include 추가 (다른 패키지에서 헤더 쓸 수 있게)
## -----------------------------------------------------------------------------
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    std_msgs
    sensor_msgs
    visualization_msgs
    morai_msgs
    rosbag
    laser_geometry
)

## -----------------------------------------------------------------------------
## Include dirs
##  - include/autorace_control/*.hpp 를 찾게 해 줌
## -----------------------------------------------------------------------------
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## -----------------------------------------------------------------------------
## Executables
## -----------------------------------------------------------------------------

# ego_path_recorder는 소스 파일이 없으므로 잠시 비활성화
# add_executable(ego_path_recorder src/ego_path_recorder.cpp)
# target_compile_features(ego_path_recorder PRIVATE cxx_std_11)
# target_link_libraries(ego_path_recorder ${catkin_LIBRARIES})

# add_executable(controller_from_center_wego_ src/controller_from_center_wego_.cpp)
# target_compile_features(controller_from_center_wego_ PRIVATE cxx_std_11)
# target_link_libraries(controller_from_center_wego_ ${catkin_LIBRARIES})


add_executable(controller_lane_logic
  src/controller_lane_logic.cpp
)
target_compile_features(controller_lane_logic PRIVATE cxx_std_11)
target_link_libraries(controller_lane_logic
  ${catkin_LIBRARIES}
)
