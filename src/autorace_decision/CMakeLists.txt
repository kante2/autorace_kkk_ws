cmake_minimum_required(VERSION 3.0.2)
project(autorace_decision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs
  morai_msgs
  rosbag
  visualization_msgs
  autorace_perception
  laser_geometry
  autorace_control # control부 패키지 추가,,
)


catkin_package(
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    std_msgs
    morai_msgs
    visualization_msgs
    autorace_perception
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


# add_executable(ego_path_recorder_bag_kante src/ego_path_recorder_bag_kante.cpp)
# target_compile_features(ego_path_recorder_bag_kante PRIVATE cxx_std_11)
# target_link_libraries(ego_path_recorder_bag_kante ${catkin_LIBRARIES})

# add_executable(ego_path_recorder_csv_kante src/ego_path_recorder_csv_kante.cpp)
# target_compile_features(ego_path_recorder_csv_kante PRIVATE cxx_std_11)
# target_link_libraries(ego_path_recorder_csv_kante ${catkin_LIBRARIES})

add_executable(main_node src/main_node.cpp)
target_compile_features(main_node PRIVATE cxx_std_11)
target_link_libraries(main_node ${catkin_LIBRARIES})



