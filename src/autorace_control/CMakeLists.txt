cmake_minimum_required(VERSION 3.0.2)
project(autorace_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs
  morai_msgs
  rosbag
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    std_msgs
    morai_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ego_path_recorder는 소스 파일이 없으므로 잠시 비활성화
# add_executable(ego_path_recorder src/ego_path_recorder.cpp)
# target_compile_features(ego_path_recorder PRIVATE cxx_std_11)
# target_link_libraries(ego_path_recorder ${catkin_LIBRARIES})

#add_executable(controller_from_center_wego_ src/controller_from_center_wego_.cpp)
#target_compile_features(controller_from_center_wego_ PRIVATE cxx_std_11)
#target_link_libraries(controller_from_center_wego_ ${catkin_LIBRARIES})

add_executable(controller_lane_logic src/controller_lane_logic.cpp)
target_compile_features(controller_lane_logic PRIVATE cxx_std_11)
target_link_libraries(controller_lane_logic ${catkin_LIBRARIES})