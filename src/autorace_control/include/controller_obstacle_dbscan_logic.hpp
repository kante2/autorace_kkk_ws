// include/autorace_control/controller_obstacle_dbscan.hpp
#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <laser_geometry/laser_geometry.h>
#include <vector>

// 장애물 포인트 2D 구조체
struct Point2D
{
  double x;
  double y;
};

// ===== 이 노드 전용 전역 (extern) =====

// 퍼블리셔 (Marker / PointCloud2)
// 정의는 controller_obstacle_dbscan_node.cpp 안에서 함
extern ros::Publisher g_pub_marker_obs;
extern ros::Publisher g_pub_cloud_obs;

// LaserProjection (scan -> PointCloud2)
extern laser_geometry::LaserProjection g_projector_obs;

// ===== 파라미터 전역 (extern) =====
extern double g_eps;
extern int    g_min_samples;
extern double g_range_min;
extern double g_range_max;
extern double g_front_min_deg;
extern double g_front_max_deg;

extern double g_follow_speed_mps;
extern double g_min_speed_mps;
extern double g_k_yaw;

// ===== 함수 프로토타입 =====

// 이 노드 전용 파라미터 로딩
void loadObstacleParams(ros::NodeHandle& pnh);

// DBSCAN 클러스터링
std::vector<int> dbscan(const std::vector<Point2D>& points);

// No target일 때 최소 속도(천천히 전진 or 정지)
void publishMinimalSpeed(bool move_forward = true);

// LaserScan 콜백
void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);
