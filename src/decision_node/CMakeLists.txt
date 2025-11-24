cmake_minimum_required(VERSION 3.0.2)
project(decision_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  morai_msgs
  rosbag
  image_transport
  cv_bridge
  visualization_msgs
  autorace_perception
  laser_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
  # autorace_control  # <- autorace_control 헤더/라이브러리 직접 쓸 거면 이 줄도 다시 추가
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
  std_msgs
  sensor_msgs
  morai_msgs
  visualization_msgs
  image_transport
  cv_bridge
  autorace_perception
  autorace_control   # ★ 네가 적어둔 것 유지
  rosbag             # ★ 유지
  laser_geometry     # ★ 유지
  tf2
  tf2_ros
  tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ★ main_node 안에 미션 소스들 전부 포함
add_executable(main_node
  src/main_node.cpp
  src/mission_lane/mission_lane.cpp
  src/mission_labacorn/mission_labacorn.cpp
  # ----------
  src/mission_gate/mission_gate.cpp
  src/mission_crosswalk/mission_crosswalk.cpp
  src/mission_rotary/mission_rotary.cpp
  src/mission_parking/mission_parking.cpp
  # 필요하면 나중에 이런 것도 추가 가능
  # src/mission_parking.cpp
  # src/mission_rotary.cpp
  # src/mission_AB.cpp
  # ...
)

target_compile_features(main_node PRIVATE cxx_std_17)

add_dependencies(main_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(main_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 디버그용 crosswalk 단독 노드
add_executable(crosswalk_main
  src/mission_crosswalk/crosswalk_debug.cpp
  src/mission_crosswalk/mission_crosswalk.cpp
)

target_compile_features(crosswalk_main PRIVATE cxx_std_11)

add_dependencies(crosswalk_main
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(crosswalk_main
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 디버그용 rotary 단독 노드
add_executable(rotary_main
  src/mission_rotary/rotary_debug.cpp
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

# 디버그용 parking 단독 노드
add_executable(parking_main
  src/mission_parking/parking_debug.cpp
  src/mission_parking/mission_parking.cpp
)

target_compile_features(parking_main PRIVATE cxx_std_17)

add_dependencies(parking_main
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(parking_main
  ${catkin_LIBRARIES}
)

# 디버그용 gate 단독 노드
add_executable(gate_debug
  src/mission_gate/gate_debug.cpp
  src/mission_gate/mission_gate.cpp
)

target_compile_features(gate_debug PRIVATE cxx_std_11)

add_dependencies(gate_debug
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(gate_debug
  ${catkin_LIBRARIES}
)
