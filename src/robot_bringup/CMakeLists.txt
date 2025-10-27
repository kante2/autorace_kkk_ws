cmake_minimum_required(VERSION 3.0.2)
project(robot_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# odom_tf_broadcaster: subscribes to /wheel_odom (nav_msgs/Odometry) and publishes TF (odom->base_link)
add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)
target_link_libraries(odom_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(odom_tf_broadcaster ${catkin_EXPORTED_TARGETS})

# com_from_imu_node: processes IMU data to estimate center of mass state
add_executable(com_from_imu_node src/com_from_imu_node.cpp)
target_link_libraries(com_from_imu_node ${catkin_LIBRARIES})
add_dependencies(com_from_imu_node ${catkin_EXPORTED_TARGETS})

install(TARGETS odom_tf_broadcaster com_from_imu_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
