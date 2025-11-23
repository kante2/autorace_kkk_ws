#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <laser_geometry/laser_geometry.h>

#include <cmath>
#include <vector>
#include <string>
#include <limits>
#include <algorithm>

// -------------------- 유틸 --------------------
inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- 2D 포인트 --------------------
struct Point2D
{
  double x;
  double y;
};

// -------------------- 전역 토픽/퍼블리셔 --------------------
std::string g_topic_scan;
std::string g_topic_cloud;
std::string g_topic_marker;
std::string g_topic_detected;

ros::Publisher g_pub_cloud;
ros::Publisher g_pub_marker;
ros::Publisher g_pub_detected;

laser_geometry::LaserProjection g_projector;

// -------------------- 전역 파라미터 --------------------
// DBSCAN / ROI 파라미터
double g_eps           = 0.2;
int    g_min_samples   = 3;
double g_range_min     = 0.13;
double g_range_max     = 1.5;
double g_front_min_deg = 60.0;
double g_front_max_deg = 300.0;

// 감지 거리 (이 값 이내에 클러스터가 있으면 labacorn_detected = true)
double g_detect_distance = 0.5; // [m] 0.5

// -------------------- DBSCAN --------------------
std::vector<int> dbscan(const std::vector<Point2D>& points)
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

    // i의 이웃 찾기
    std::vector<int> neighbors;
    for (int j = 0; j < n; ++j)
    {
      if (dist(points[i], points[j]) <= g_eps)
        neighbors.push_back(j);
    }

    if ((int)neighbors.size() < g_min_samples)
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
        if (dist(points[j], points[m]) <= g_eps)
          j_neighbors.push_back(m);
      }

      if ((int)j_neighbors.size() >= g_min_samples)
      {
        seed_set.insert(seed_set.end(), j_neighbors.begin(), j_neighbors.end());
      }
    }
  }

  return labels;
}

// -------------------- 1. 스캔 포인트 필터링 --------------------
std::vector<Point2D> filterScanPoints(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  std::vector<Point2D> points;
  points.reserve(scan->ranges.size());

  double angle = scan->angle_min;

  for (auto r : scan->ranges)
  {
    // 거리 필터
    if (!std::isfinite(r) || r < g_range_min || r > g_range_max)
    {
      angle += scan->angle_increment;
      continue;
    }

    // 각도 필터 (degree 기준)
    double angle_deg = angle * 180.0 / M_PI;
    if (angle_deg < 0.0) angle_deg += 360.0;

    if (angle_deg < g_front_min_deg || angle_deg > g_front_max_deg)
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

// -------------------- 2. 클러스터 감지 및 Marker 퍼블리시 --------------------
bool detectClustersWithinDistance(const std::vector<Point2D>& points,
                                  const std::string& frame_id,
                                  const ros::Time& stamp)
{
  if (points.empty())
    return false;

  std::vector<int> labels = dbscan(points);
  if (labels.empty())
    return false;

  int max_label = *std::max_element(labels.begin(), labels.end());
  if (max_label < 1)
    return false;  // 전부 노이즈(-1)인 경우

  bool detected = false;
  int marker_id = 0;

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

    double d = std::hypot(cx, cy);

    // Marker (디버그용)
    if (g_pub_marker.getNumSubscribers() > 0)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp    = stamp;
      marker.ns   = "labacorn_clusters";
      marker.id   = marker_id++;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = cx;
      marker.pose.position.y = cy;
      marker.pose.position.z = 0.0;

      marker.scale.x = 0.15;
      marker.scale.y = 0.15;
      marker.scale.z = 0.15;

      marker.color.r = 1.0;
      marker.color.g = 0.5;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration(0.2);
      g_pub_marker.publish(marker);
    }

    // 감지 거리 이내인지 확인
    if (d <= g_detect_distance)
    {
      detected = true;
    }
  }

  return detected;
}

// -------------------- 스캔 콜백 --------------------
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // PointCloud2 디버그용 퍼블리시 (옵션)
  if (g_pub_cloud.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 cloud;
    g_projector.projectLaser(*scan, cloud);
    cloud.header = scan->header;
    g_pub_cloud.publish(cloud);
  }

  // 1) ROI 필터링
  std::vector<Point2D> points = filterScanPoints(scan);

  // 2) DBSCAN + 감지 여부 판단
  bool detected = detectClustersWithinDistance(points,
                                               scan->header.frame_id,
                                               scan->header.stamp);

  // 3) /labacorn_detected 퍼블리시
  std_msgs::Bool msg;
  msg.data = detected;
  g_pub_detected.publish(msg);

  ROS_INFO_THROTTLE(0.5,
      "[cluster_detect_lidar] points=%zu, detected=%d (dist<=%.2f m)",
      points.size(), (int)detected, g_detect_distance);
}

// -------------------- main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cluster_detect_lidar");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("cluster_detect_lidar (LaserScan -> /labacorn_detected)");

  // --- 파라미터 로드 ---
  pnh.param<std::string>("scan_topic",    g_topic_scan,    std::string("/scan"));
  pnh.param<std::string>("cloud_topic",   g_topic_cloud,   std::string("/scan_points"));
  pnh.param<std::string>("marker_topic",  g_topic_marker,  std::string("/labacorn_clusters"));
  pnh.param<std::string>("detected_topic", g_topic_detected,
                         std::string("/labacorn_detected"));

  // DBSCAN / ROI
  pnh.param<double>("eps",           g_eps,           0.2); // CORN간 이웃하는 대상 간의 거리 -> 이하이면 이웃으로 판단하는 값이다. 
  pnh.param<int>   ("min_samples",   g_min_samples,   3);   // 3개 이상인 이웃관계가 있으면 -> 하나의 덩어리로 보는 파라미터.  
  pnh.param<double>("range_min",     g_range_min,     0.13);
  pnh.param<double>("range_max",     g_range_max,     1.5);
  pnh.param<double>("front_min_deg", g_front_min_deg, 60.0);
  pnh.param<double>("front_max_deg", g_front_max_deg, 300.0);

  // 감지 거리 (기본 0.5m)
  pnh.param<double>("detect_distance", g_detect_distance, 0.5);

  ROS_INFO("[cluster_detect_lidar] subscribe scan='%s'",
           ros::names::resolve(g_topic_scan).c_str());
  ROS_INFO("[cluster_detect_lidar] publish detected='%s', marker='%s', cloud='%s'",
           ros::names::resolve(g_topic_detected).c_str(),
           ros::names::resolve(g_topic_marker).c_str(),
           ros::names::resolve(g_topic_cloud).c_str());

  // --- Pub/Sub ---
  g_pub_cloud    = nh.advertise<sensor_msgs::PointCloud2>(g_topic_cloud,    1);
  g_pub_marker   = nh.advertise<visualization_msgs::Marker>(g_topic_marker, 10);
  g_pub_detected = nh.advertise<std_msgs::Bool>(g_topic_detected, 10);

  ros::Subscriber scan_sub = nh.subscribe(g_topic_scan, 1, scanCallback);

  ros::spin();
  return 0;
}
