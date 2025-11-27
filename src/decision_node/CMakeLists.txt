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

# ★ main_node (C++ 버전) — src/main_node.cpp가 있을 때만 빌드
set(MAIN_NODE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/main_node.cpp)
if(EXISTS ${MAIN_NODE_SRC})
  add_executable(main_node
    src/main_node.cpp
    src/mission_lane/mission_lane.cpp
    src/mission_labacorn/mission_labacorn.cpp
    # ----------
    # 디버그용 파일들 삭제
    # src/mission_lane/lane_debug.cpp
    # src/mission_gate/gate_debug.cpp
    # src/mission_ab/ab_left_debug.cpp
    src/mission_gate/mission_gate.cpp
    src/mission_crosswalk/mission_crosswalk.cpp
    src/mission_rotary/mission_rotary.cpp
    src/mission_parking/mission_parking.cpp
    src/mission_ab/mission_ab_left.cpp
    src/mission_ab/mission_ab_right.cpp
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
else()
  message(WARNING "decision_node: src/main_node.cpp not found; skipping main_node target")
endif()

# 센서 enable/하드코딩 타이머 노드 (C++ 버전이 있을 때만 빌드; 기본은 python 스크립트 사용)
set(MISSION_NODE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/mission_node.cpp)
if(EXISTS ${MISSION_NODE_SRC})
  add_executable(mission_node
    src/mission_node.cpp
  )

  target_compile_features(mission_node PRIVATE cxx_std_17)

  add_dependencies(mission_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(mission_node
    ${catkin_LIBRARIES}
  )
else()
  message(WARNING "decision_node: src/mission_node.cpp not found; skipping mission_node target (use mission_node_pyt.py)")
endif()

# bag enable 신호에 따라 rosbag play를 트리거하는 노드
add_executable(mission_bag_play
  src/mission_bag_play.cpp
)

target_compile_features(mission_bag_play PRIVATE cxx_std_17)

add_dependencies(mission_bag_play
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mission_bag_play
  ${catkin_LIBRARIES}
)

# 차선 추종만 단독으로 실행할 수 있는 간단한 노드
add_executable(lane_solo
  src/mission_lane/lane_solo.cpp
  src/mission_lane/mission_lane.cpp
)

target_compile_features(lane_solo PRIVATE cxx_std_17)

add_dependencies(lane_solo
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(lane_solo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 라바콘 미션 단독 실행 노드
add_executable(labacorn_solo
  src/mission_labacorn/labacorn_solo.cpp
  src/mission_labacorn/mission_labacorn.cpp
)

target_compile_features(labacorn_solo PRIVATE cxx_std_17)

add_dependencies(labacorn_solo
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(labacorn_solo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 로터리 미션 단독 실행 노드
add_executable(rotary_solo
  src/mission_rotary/roatry_solo.cpp
  src/mission_rotary/mission_rotary.cpp
)

target_compile_features(rotary_solo PRIVATE cxx_std_17)

add_dependencies(rotary_solo
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(rotary_solo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
