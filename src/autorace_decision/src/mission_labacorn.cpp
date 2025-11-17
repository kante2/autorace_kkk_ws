// src/controller_obstacle_dbscan_node.cpp

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "autorace_control/controller_lane_logic.hpp"        // loadParams, publish_motor_commands, 전역 extern 등
#include "autorace_control/controller_obstacle_dbscan.hpp"   // 장애물 DBSCAN 관련

// ===== lane_logic.hpp에서 extern 선언된 전역들의 실제 정의 =====
// (실제 프로젝트에 맞춰 이미 다른 cpp에 정의되어 있다면, 이 부분은 조정 필요)
ros::Publisher g_pub_motor;
ros::Publisher g_pub_servo;

// 토픽 이름 전역 (lane_logic.hpp 안에 extern으로 선언되어 있다고 가정)
std::string g_motor_topic;
std::string g_servo_topic;

// 그 외 lane_logic.hpp 안에서 사용하는 파라미터 전역들도
// 보통 거기에 extern으로 되어 있을 테니, 이미 다른 cpp에서 정의 중이면
// 여기서 다시 정의하면 안 됨. (현재 구조에 맞게 조정해줘야 함)

// 이 노드 로컬용 subscriber
ros::Subscriber g_sub_scan_obs;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller_obstacle_dbscan");
  ros::NodeHandle nh;

  // 1) lane 공통 파라미터 로딩 (servo/motor 스케일 등)
  loadParams(nh);

  // 2) 장애물 전용 파라미터 로딩
  loadObstacleParams(nh);

  // 3) 모터/서보 퍼블리셔 생성 (lane 공통 토픽 사용)
  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 1);
  g_pub_servo = nh.advertise<std_msgs::Float64>(g_servo_topic, 1);

  // 4) 장애물 전용 퍼블리셔 (Marker / PointCloud2)
  g_pub_marker_obs = nh.advertise<visualization_msgs::Marker>("/dbscan_lines", 300);
  g_pub_cloud_obs  = nh.advertise<sensor_msgs::PointCloud2>("/scan_points", 300);

  // 5) LaserScan 구독
  g_sub_scan_obs = nh.subscribe("/scan", 1, scanCallback);

  ROS_INFO("[controller_obstacle_dbscan] node started");
  ros::spin();
  return 0;
}
