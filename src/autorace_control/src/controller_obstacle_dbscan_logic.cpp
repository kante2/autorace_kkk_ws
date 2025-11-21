#include "autorace_control/controller_obstacle_dbscan_logic.hpp"

#include <laser_geometry/laser_geometry.h>
#include <cmath>
#include <vector>
#include <algorithm>

// -------------------- 유틸 --------------------
inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- 내부 전용 구조체 --------------------
struct Point2D
{
  double x;
  double y;
};

struct ClusterCenters
{
  double left_x_sum  = 0.0;
  double left_y_sum  = 0.0;
  double right_x_sum = 0.0;
  double right_y_sum = 0.0;
  int    left_cnt    = 0;
  int    right_cnt   = 0;
};

// -------------------- 내부 전용 객체 --------------------
static laser_geometry::LaserProjection s_obs_projector;

// -------------------- DBSCAN 구현 --------------------
static std::vector<int> dbscan(const std::vector<Point2D>& points,
                               double eps,
                               int min_samples)
{
  const int n = static_cast<int>(points.size());
  std::vector<int> labels(n, -1);
  if (n == 0) return labels;

  int cluster_id = 0;

  auto dist = [](const Point2D& a, const Point2D& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  };

  for (int i = 0; i < n; ++i)
  {
    if (labels[i] != -1) continue;

    std::vector<int> neighbors;
    for (int j = 0; j < n; ++j)
    {
      if (dist(points[i], points[j]) <= eps)
        neighbors.push_back(j);
    }

    if ((int)neighbors.size() < min_samples)
      continue;  // 노이즈

    ++cluster_id;
    labels[i] = cluster_id;

    std::vector<int> seed_set = neighbors;
    for (size_t k = 0; k < seed_set.size(); ++k)
    {
      int j = seed_set[k];
      if (labels[j] != -1) continue;

      labels[j] = cluster_id;

      std::vector<int> j_neighbors;
      for (int m = 0; m < n; ++m)
      {
        if (dist(points[j], points[m]) <= eps)
          j_neighbors.push_back(m);
      }

      if ((int)j_neighbors.size() >= min_samples)
      {
        seed_set.insert(seed_set.end(), j_neighbors.begin(), j_neighbors.end());
      }
    }
  }

  return labels;
}

// -------------------- 1) 스캔 필터링 --------------------
static std::vector<Point2D> filterScanPoints(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  std::vector<Point2D> points;
  points.reserve(scan->ranges.size());

  double angle = scan->angle_min;

  for (auto r : scan->ranges)
  {
    // 거리 필터
    if (!std::isfinite(r) || r < g_obs_range_min || r > g_obs_range_max)
    {
      angle += scan->angle_increment;
      continue;
    }

    // 각도 필터 (degree)
    double angle_deg = angle * 180.0 / M_PI;
    if (angle_deg < 0.0) angle_deg += 360.0;

    if (angle_deg < g_obs_front_min_deg || angle_deg > g_obs_front_max_deg)
    {
      angle += scan->angle_increment;
      continue;
    }

    Point2D p;
    p.x = r * std::cos(angle);
    p.y = r * std::sin(angle);
    points.push_back(p);

    angle += scan->angle_increment;
  }

  return points;
}

// -------------------- 2) 클러스터 중심 계산 + Marker 퍼블리시 --------------------
static ClusterCenters computeClusterCenters(const std::vector<Point2D>& points,
                                            const std::string& frame_id,
                                            const ros::Time& stamp)
{
  ClusterCenters centers;

  if (points.empty())
    return centers;

  std::vector<int> labels = dbscan(points, g_obs_eps, g_obs_min_samples);
  if (labels.empty())
    return centers;

  int max_label = *std::max_element(labels.begin(), labels.end());
  if (max_label < 1)
    return centers;

  for (int c = 1; c <= max_label; ++c)
  {
    double cx = 0.0, cy = 0.0;
    int    count = 0;

    for (size_t i = 0; i < points.size(); ++i)
    {
      if (labels[i] == c)
      {
        cx += points[i].x;
        cy += points[i].y;
        ++count;
      }
    }
    if (count == 0) continue;

    cx /= count;
    cy /= count;

    // 좌/우 분류
    if (cy > 0.0)
    {
      centers.left_x_sum  += cx;
      centers.left_y_sum  += cy;
      centers.left_cnt    += 1;
    }
    else
    {
      centers.right_x_sum += cx;
      centers.right_y_sum += cy;
      centers.right_cnt   += 1;
    }

    // Marker 퍼블리시 (디버그)
    if (g_pub_obs_marker)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp    = stamp;
      marker.ns   = "obs_cluster";
      marker.id   = c;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = cx;
      marker.pose.position.y = cy;
      marker.pose.position.z = 0.0;

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration(0.1);
      g_pub_obs_marker.publish(marker);
    }
  }

  return centers;
}

// -------------------- 3) 목표 좌표 계산 --------------------
static bool computeTargetFromCenters(const ClusterCenters& centers,
                                     double& target_x,
                                     double& target_y)
{
  if (centers.left_cnt > 0 && centers.right_cnt > 0)
  {
    double left_x  = centers.left_x_sum  / centers.left_cnt;
    double left_y  = centers.left_y_sum  / centers.left_cnt;
    double right_x = centers.right_x_sum / centers.right_cnt;
    double right_y = centers.right_y_sum / centers.right_cnt;

    int total = centers.left_cnt + centers.right_cnt;

    target_x = (double)centers.right_cnt / total * left_x +
               (double)centers.left_cnt   / total * right_x;

    target_y = (double)centers.right_cnt / total * left_y +
               (double)centers.left_cnt   / total * right_y;

    return true;
  }
  else if (centers.left_cnt > 0)
  {
    target_x = centers.left_x_sum / centers.left_cnt;
    target_y = (centers.left_y_sum / centers.left_cnt) - g_obs_lane_offset_y;
    return true;
  }
  else if (centers.right_cnt > 0)
  {
    target_x = centers.right_x_sum / centers.right_cnt;
    target_y = (centers.right_y_sum / centers.right_cnt) + g_obs_lane_offset_y;
    return true;
  }

  return false;
}

// -------------------- 파라미터 로드 --------------------
void loadParams_obstacle_dbscan(ros::NodeHandle& pnh)
{
  // Topics
  pnh.param<std::string>("obstacle_scan_topic",
                         g_obs_scan_topic,
                         std::string("/scan"));

  pnh.param<std::string>("obstacle_marker_topic",
                         g_obs_marker_topic,
                         std::string("/dbscan_lines"));

  pnh.param<std::string>("obstacle_cloud_topic",
                         g_obs_cloud_topic,
                         std::string("/scan_points"));

  // DBSCAN / ROI
  pnh.param<double>("obstacle_eps",           g_obs_eps,            0.2);
  pnh.param<int>   ("obstacle_min_samples",   g_obs_min_samples,    4);
  pnh.param<double>("obstacle_range_min",     g_obs_range_min,      0.13);
  pnh.param<double>("obstacle_range_max",     g_obs_range_max,      0.9);
  pnh.param<double>("obstacle_front_min_deg", g_obs_front_min_deg,  60.0);
  pnh.param<double>("obstacle_front_max_deg", g_obs_front_max_deg,  300.0);
  pnh.param<double>("obstacle_lane_offset_y", g_obs_lane_offset_y,  0.12);

  // yaw -> 조향/속도
  pnh.param<double>("obstacle_yaw_gain",         g_obs_yaw_gain,           0.8);
  pnh.param<double>("obstacle_follow_speed_mps", g_obs_follow_speed_mps,   1.0);
  pnh.param<double>("obstacle_min_speed_mps",    g_obs_min_speed_mps,      0.7);

  // Timeout
  pnh.param<double>("obstacle_scan_timeout_sec", g_obs_scan_timeout_sec, 0.5);

  ROS_INFO("[obs_dbscan] loaded params: scan='%s', marker='%s', cloud='%s'",
           ros::names::resolve(g_obs_scan_topic).c_str(),
           ros::names::resolve(g_obs_marker_topic).c_str(),
           ros::names::resolve(g_obs_cloud_topic).c_str());
}

// -------------------- LaserScan 콜백 --------------------
void CB_obstacle_scan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // 콜백 시간 기록
  g_obs_last_scan_time = ros::Time::now();
  g_obs_have_scan_time = true;

  // PointCloud2 디버그용 퍼블리시
  if (g_pub_obs_cloud && g_pub_obs_cloud.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 cloud;
    s_obs_projector.projectLaser(*scan, cloud);
    cloud.header = scan->header;
    g_pub_obs_cloud.publish(cloud);
  }

  // 1) 스캔 포인트 필터링
  std::vector<Point2D> points = filterScanPoints(scan);
  if (points.empty())
  {
    g_obs_have_target = false;
    ROS_INFO_THROTTLE(1.0, "[obs_dbscan] No valid points in ROI");
    return;
  }

  // 2) 클러스터 중심 계산 + Marker
  ClusterCenters centers =
      computeClusterCenters(points,
                            scan->header.frame_id,
                            scan->header.stamp);

  // 3) 목표 좌표 계산
  double target_x = 0.0, target_y = 0.0;
  if (!computeTargetFromCenters(centers, target_x, target_y))
  {
    g_obs_have_target = false;
    ROS_INFO_THROTTLE(1.0, "[obs_dbscan] No clusters -> no target");
    return;
  }

  // 4) 목표 좌표 -> yaw -> 조향/속도
  double yaw = std::atan2(target_y, target_x);  // 앞(+x), 좌(+y)

  double steer_cmd = std::tanh(g_obs_yaw_gain * yaw); // -1~+1
  double speed_cmd = g_obs_follow_speed_mps;          // 필요시 yaw에 따라 줄이는 로직 추가 가능

  g_obs_latest_steer_cmd = steer_cmd;
  g_obs_latest_speed_cmd = speed_cmd;
  g_obs_have_target      = true;

  ROS_INFO_THROTTLE(0.5,
    "[obs_dbscan][CB] target=(%.2f, %.2f), yaw=%.2f rad -> steer=%.3f, v=%.2f",
    target_x, target_y, yaw, steer_cmd, speed_cmd);
}
