cmake_minimum_required(VERSION 3.0.2)
project(autorace_perception)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  morai_msgs
  nav_msgs
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  usb_cam
  image_proc
  image_view
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    morai_msgs
    nav_msgs
    roscpp
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    usb_cam
    image_proc
    image_view
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## (선택) 노드가 있다면 여기에 추가
# add_executable(my_node src/my_node.cpp)
# target_link_libraries(my_node ${catkin_LIBRARIES})
# add_dependencies(my_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
