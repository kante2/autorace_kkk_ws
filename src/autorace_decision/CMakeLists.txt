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
  # autorace_control  # <- autorace_control 헤더/라이브러리 직접 쓸 거면 이 줄도 다시 추가
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
    autorace_control   # ★ 네가 적어둔 것 유지
    rosbag             # ★ 유지
    laser_geometry     # ★ 유지
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# ★ main_node 안에 미션 소스들 전부 포함
add_executable(main_node
  src/main_node.cpp
  src/mission_lane_toMain.cpp
  src/mission_labacorn_toMain.cpp
  src/mission_crosswalk_toMain.cpp
  src/mission_gate_toMain.cpp
  src/mission_AB_left_toMain.cpp
  src/mission_AB_right_toMain.cpp
  src/mission_rotary_toMain.cpp
  src/mission_color_toMain.cpp
)

target_compile_features(main_node PRIVATE cxx_std_11)

add_dependencies(main_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(main_node
  ${catkin_LIBRARIES}
)

# 디버그용 rotary 단독 노드
add_executable(rotary_main
  src/mission_rotary/rotary_main.cpp
  src/mission_rotary/mission_rotary.cpp
)

target_compile_features(rotary_main PRIVATE cxx_std_11)

add_dependencies(rotary_main
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(rotary_main
  ${catkin_LIBRARIES}
)
