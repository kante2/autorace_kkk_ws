cmake_minimum_required(VERSION 3.0.2)
project(autorace_perception)

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

## msg file
add_message_files(
  FILES
  GateCluster.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

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
# 사용 예시:
#   add_wego_node(lane_kante_wego_node src/lane_kante_wego.cpp)
#   add_wego_node(lane_jun_wego_node   src/lane_jun_wego.cpp)
#
function(add_wego_node node_name src_file)
  add_executable(${node_name}
    ${src_file}
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

# === lane_kante_wego ===
add_wego_node(lane_kante_wego_node src/lane_kante_wego.cpp)

# === lane_jun_wego ===
add_wego_node(lane_jun_wego_node   src/lane_jun_wego.cpp)

# === mission7 게이트 perception (dbscan_gate.cpp) ===
add_wego_node(mission7_gate_perception_node src/dbscan_gate.cpp)

## 나중에 새 노드 추가할 때는 밑에 한 줄만 추가:
## add_wego_node(새노드이름_node src/새파일이름.cpp)

#####################
## 설치 규칙 (옵션) ##
#####################
install(TARGETS
  lane_kante_wego_node
  lane_jun_wego_node
  mission7_gate_perception_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
