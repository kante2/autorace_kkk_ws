// src/controller_obstacle_dbscan_logic.cpp

#include <cmath>
#include <algorithm>

#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <laser_geometry/laser_geometry.h>

#include "autorace_control/controller_lane_logic.hpp"        // clamp, publish_motor_commands 등
#include "autorace_control/controller_obstacle_dbscan.hpp"   // 이 파일에 선언된 extern/함수

// ===== 전역 정의 (헤더의 extern들 실제 정의) =====

// 퍼블리셔
ros::Publisher g_pub_marker_obs;
ros::Publisher g_pub_cloud_obs;

// LaserProjection
laser_geometry::LaserProjection g_projector_obs;

// 파라미터
double g_eps            = 0.2;
int    g_min_samples    = 2;
double g_range_min      = 0.13;
double g_range_max      = 0.90;
double g_front_min_deg  = 60.0;
double g_front_max_deg  = 300.0;

double g_follow_speed_mps = 1.0;
double g_min_speed_mps    = 0.3;
double g_k_yaw            = 1.2;

// ===== 파라미터 로딩 =====
void loadObstacleParams(ros::NodeHandle& pnh)
{
  pnh.param<double>("eps",            g_eps,            0.2);
  pnh.param<int   >("min_samples",    g_min_samples,    2);
  pnh.param<double>("range_min",      g_range_min,      0.13);
  pnh.param<double>("range_max",      g_range_max,      0.90);
  pnh.param<double>("front_min_deg",  g_front_min_deg,  60.0);
  pnh.param<double>("front_max_deg",  g_front_max_deg,  300.0);

  pnh.param<double>("follow_speed_mps", g_follow_speed_mps, 1.0);
  pnh.param<double>("min_speed_mps",    g_min_speed_mps,    0.3);
  pnh.param<double>("k_yaw",            g_k_yaw,            1.2);

  ROS_INFO("[obstacle_dbscan] eps=%.2f, min_samples=%d, range=[%.2f, %.2f], "
           "front=[%.1f, %.1f], v_follow=%.2f, v_min=%.2f, k_yaw=%.2f",
           g_eps, g_min_samples, g_range_min, g_range_max,
           g_front_min_deg, g_front_max_deg,
           g_follow_speed_mps, g_min_speed_mps, g_k_yaw);
}

// ===== DBSCAN 구현 =====
std::vector<int> dbscan(const std::vector<Point2D>& points)
{
  int n = static_cast<int>(points.size());
  std::vector<int> labels(n, -1);
  int cluster_id = 0;

  auto dist = [](const Point2D& a, const Point2D& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  };

  for (int i = 0; i < n; ++i)
  {
    if (labels[i] != -1) continue;

    std::vector<int> neighbors;
    neighbors.reserve(n);

    for (int j = 0; j < n; ++j)
    {
      if (dist(points[i], points[j]) <= g_eps)
        neighbors.push_back(j);
    }

    if (static_cast<int>(neighbors.size()) < g_min_samples)
      continue; // 잡음

    ++cluster_id;
    labels[i] = cluster_id;

    std::vector<int> seed_set = neighbors;
    for (size_t k = 0; k < seed_set.size(); ++k)
    {
      int j = seed_set[k];
      if (labels[j] != -1) continue;

      labels[j] = cluster_id;

      std::vector<int> j_neighbors;
      j_neighbors.reserve(n);
      for (int m = 0; m < n; ++m)
      {
        if (dist(points[j], points[m]) <= g_eps)
          j_neighbors.push_back(m);
      }

      if (static_cast<int>(j_neighbors.size()) >= g_min_samples)
      {
        seed_set.insert(seed_set.end(),
                        j_neighbors.begin(), j_neighbors.end());
      }
    }
  }
  return labels;
}

// ===== No target일 때 최소 속도 =====
void publishMinimalSpeed(bool move_forward)
{
  double steer_cmd = 0.0;
  double speed_cmd = 0.0;

  if (move_forward)
  {
    speed_cmd = g_min_speed_mps;  // 천천히 전진
    ROS_INFO_THROTTLE(1.0,
      "[obstacle_dbscan] No target -> move forward slowly (v=%.2f)",
      speed_cmd);
  }
  else
  {
    speed_cmd = 0.0;              // 완전 정지
    ROS_INFO_THROTTLE(1.0,
      "[obstacle_dbscan] No target -> STOP");
  }

  // lane_logic.hpp 의 공통 함수 사용
  publish_motor_commands(steer_cmd, speed_cmd);
}

// ===== LaserScan 콜백 =====
void scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  std::vector<Point2D> points;
  points.reserve(scan->ranges.size());

  double angle = scan->angle_min;

  // --- 포인트 필터링: 거리 + 전방 각도 ---
  for (const auto& r : scan->ranges)
  {
    if (!std::isfinite(r) || r < g_range_min || r > g_range_max)
    {
      angle += scan->angle_increment;
      continue;
    }

    double angle_deg = angle * 180.0 / M_PI;
    if (angle_deg < 0.0) angle_deg += 360.0;

    if (angle_deg < g_front_min_deg || angle_deg > g_front_max_deg)
    {
      angle += scan->angle_increment;
      continue;
    }

    double x = r * std::cos(angle);
    double y = r * std::sin(angle);
    points.push_back({x, y});

    angle += scan->angle_increment;
  }

  if (points.empty())
  {
    ROS_WARN_THROTTLE(1.0,
      "[obstacle_dbscan] No valid points -> minimal speed");
    publishMinimalSpeed(true);
    return;
  }

  // --- PointCloud2 퍼블리시 (디버깅용) ---
  if (g_pub_cloud_obs)
  {
    sensor_msgs::PointCloud2 cloud;
    g_projector_obs.projectLaser(*scan, cloud);
    cloud.header.stamp = scan->header.stamp;
    cloud.header.frame_id = scan->header.frame_id;
    g_pub_cloud_obs.publish(cloud);
  }

  // --- DBSCAN 클러스터링 ---
  std::vector<int> labels = dbscan(points);
  int max_label = 0;
  if (!labels.empty())
    max_label = *std::max_element(labels.begin(), labels.end());

  ROS_INFO_THROTTLE(1.0,
    "[obstacle_dbscan] Clusters detected: %d", max_label);

  double left_x_sum  = 0.0, left_y_sum  = 0.0;
  double right_x_sum = 0.0, right_y_sum = 0.0;
  int left_cnt = 0, right_cnt = 0;

  // --- 클러스터 중심 계산 + Marker ---
  for (int c = 1; c <= max_label; ++c)
  {
    double cx = 0.0, cy = 0.0;
    int count = 0;

    for (size_t i = 0; i < points.size(); ++i)
    {
      if (labels[i] == c)
      {
        cx += points[i].x;
        cy += points[i].y;
        ++count;
      }
    }

    if (count <= 0) continue;

    cx /= static_cast<double>(count);
    cy /= static_cast<double>(count);

    // y>0 : 왼쪽, y<0 : 오른쪽
    if (cy > 0.0)
    {
      ++left_cnt;
      left_x_sum  += cx;
      left_y_sum  += cy;
    }
    else
    {
      ++right_cnt;
      right_x_sum += cx;
      right_y_sum += cy;
    }

    if (g_pub_marker_obs)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = scan->header.frame_id;
      marker.header.stamp    = ros::Time::now();
      marker.ns   = "cluster";
      marker.id   = c;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = cx;
      marker.pose.position.y = cy;
      marker.pose.position.z = 0.0;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration(0.1);
      g_pub_marker_obs.publish(marker);
    }
  }

  // --- 목표 좌표 계산 ---
  double target_x = 0.0;
  double target_y = 0.0;
  double speed_cmd = g_follow_speed_mps;

  if (left_cnt > 0 && right_cnt > 0)
  {
    // 양쪽 장애물 → 가운데로 통과
    double left_x  = left_x_sum  / static_cast<double>(left_cnt);
    double left_y  = left_y_sum  / static_cast<double>(left_cnt);
    double right_x = right_x_sum / static_cast<double>(right_cnt);
    double right_y = right_y_sum / static_cast<double>(right_cnt);

    int total_cnt = left_cnt + right_cnt;
    double weight_left  = static_cast<double>(right_cnt) / total_cnt;
    double weight_right = static_cast<double>(left_cnt)  / total_cnt;

    target_x = weight_left  * left_x  + weight_right * right_x;
    target_y = weight_left  * left_y  + weight_right * right_y;
  }
  else if (left_cnt > 0)
  {
    // 왼쪽에만 장애물 → 오른쪽으로 피하기
    target_x = left_x_sum / static_cast<double>(left_cnt);
    target_y = (left_y_sum / static_cast<double>(left_cnt)) - 0.12;
  }
  else if (right_cnt > 0)
  {
    // 오른쪽에만 장애물 → 왼쪽으로 피하기
    target_x = right_x_sum / static_cast<double>(right_cnt);
    target_y = (right_y_sum / static_cast<double>(right_cnt)) + 0.12;
  }
  else
  {
    ROS_WARN_THROTTLE(1.0,
      "[obstacle_dbscan] No clusters -> minimal speed");
    publishMinimalSpeed(true);
    return;
  }

  // --- yaw 계산 & 조향 명령 ---
  double yaw = std::atan2(target_y, target_x);   // [rad]
  double steer_cmd = g_k_yaw * yaw;              // 비례 조향
  steer_cmd = clamp(steer_cmd, -1.0, 1.0);       // -1 ~ +1

  // (옵션) yaw 크기에 따라 속도를 줄이고 싶으면 아래 주석 풀어서 튜닝
  // double speed_scale = 1.0 - 0.5 * std::min(std::fabs(yaw) / (M_PI/4.0), 1.0);
  // speed_cmd = g_follow_speed_mps * speed_scale;
  // speed_cmd = clamp(speed_cmd, g_min_speed_mps, g_follow_speed_mps);

  // --- 최종 모터/서보 퍼블리시 ---
  publish_motor_commands(steer_cmd, speed_cmd);

  ROS_INFO_THROTTLE(0.5,
    "[obstacle_dbscan] v=%.2f m/s, steer_cmd=%.3f, target=(%.2f, %.2f), yaw=%.2f deg, L=%d R=%d",
    speed_cmd, steer_cmd, target_x, target_y, yaw * 180.0 / M_PI, left_cnt, right_cnt);
}
