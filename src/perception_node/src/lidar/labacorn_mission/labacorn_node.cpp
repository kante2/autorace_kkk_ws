#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <laser_geometry/laser_geometry.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "object_detect_laba.hpp"  // Point2D, extern ROI 파라미터

// -------------------- 전역 파라미터/토픽 --------------------
std::string g_scan_topic;
std::string g_marker_topic;
std::string g_cloud_topic;
std::string g_detected_topic;

double g_eps = 0.2;
int    g_min_samples = 4;
double g_range_min = 0.13;
double g_range_max = 0.9;
double g_front_min_deg = 60.0;
double g_front_max_deg = 300.0;

// -------------------- 전역 Pub --------------------
ros::Publisher g_pub_marker_arr;
ros::Publisher g_pub_cloud;
ros::Publisher g_pub_detected;
laser_geometry::LaserProjection g_projector;

// 전처리 함수 (정의는 lidar_preprocessing_labacorn.cpp)
std::vector<Point2D> filterScanPoints(const sensor_msgs::LaserScan::ConstPtr& scan);

// -------------------- DBSCAN --------------------
std::vector<int> dbscan(const std::vector<Point2D>& points)
{
  const int n = static_cast<int>(points.size());
  std::vector<int> labels(n, -1);
  if (n == 0) return labels;

  int cluster_id = 0;
  auto dist = [](const Point2D& a, const Point2D& b) { return std::hypot(a.x - b.x, a.y - b.y); };

  for (int i = 0; i < n; ++i)
  {
    if (labels[i] != -1) continue;

    std::vector<int> neighbors;
    for (int j = 0; j < n; ++j)
    {
      if (dist(points[i], points[j]) <= g_eps)
        neighbors.push_back(j);
    }

    if (static_cast<int>(neighbors.size()) < g_min_samples)
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
      if (static_cast<int>(j_neighbors.size()) >= g_min_samples)
      {
        seed_set.insert(seed_set.end(), j_neighbors.begin(), j_neighbors.end());
      }
    }
  }
  return labels;
}

// -------------------- centroid 계산 --------------------
std::vector<Point2D> computeCentroids(const std::vector<Point2D>& points,
                                      const std::vector<int>& labels)
{
  std::vector<Point2D> centroids;
  if (points.size() != labels.size()) return centroids;

  int max_label = 0;
  for (int l : labels) if (l > max_label) max_label = l;
  if (max_label < 1) return centroids;

  centroids.resize(max_label + 1);
  std::vector<int> counts(max_label + 1, 0);

  for (std::size_t i = 0; i < points.size(); ++i)
  {
    int lab = labels[i];
    if (lab <= 0) continue;
    centroids[lab].x += points[i].x;
    centroids[lab].y += points[i].y;
    counts[lab] += 1;
  }

  std::vector<Point2D> out;
  for (int c = 1; c <= max_label; ++c)
  {
    if (counts[c] == 0) continue;
    out.push_back({centroids[c].x / counts[c], centroids[c].y / counts[c]});
  }
  return out;
}

// -------------------- 스캔 콜백 --------------------
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // 디버그용 포인트클라우드
  if (g_pub_cloud.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 cloud;
    g_projector.projectLaser(*scan, cloud);
    cloud.header = scan->header;
    g_pub_cloud.publish(cloud);
  }

  // 1) ROI 필터링
  std::vector<Point2D> points = filterScanPoints(scan);

  // 2) DBSCAN + centroid 계산
  std::vector<int> labels = dbscan(points);
  std::vector<Point2D> centroids = computeCentroids(points, labels);
  bool detected = !centroids.empty();

  // 3) MarkerArray 퍼블리시
  if (g_pub_marker_arr.getNumSubscribers() > 0)
  {
    visualization_msgs::MarkerArray ma;
    int marker_id = 0;
    for (const auto& c : centroids)
    {
      visualization_msgs::Marker m;
      m.header = scan->header;
      m.ns = "dbscan_lines";
      m.id = marker_id++;
      m.type = visualization_msgs::Marker::SPHERE;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.position.x = c.x;
      m.pose.position.y = c.y;
      m.pose.position.z = 0.0;
      m.scale.x = m.scale.y = m.scale.z = 0.1;
      m.color.r = 0.0f;
      m.color.g = 1.0f;
      m.color.b = 0.0f;
      m.color.a = 1.0f;
      m.lifetime = ros::Duration(0.2);
      ma.markers.push_back(m);
    }
    g_pub_marker_arr.publish(ma);
  }

  // 4) 감지 퍼블리시
  std_msgs::Bool msg;
  msg.data = detected;
  g_pub_detected.publish(msg);

  ROS_INFO_THROTTLE(0.5,
                    "[labacorn_node] points=%zu centroids=%zu detected=%d",
                    points.size(),
                    centroids.size(),
                    static_cast<int>(detected));
}

// -------------------- main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "labacorn_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<std::string>("scan_topic", g_scan_topic, std::string("/scan"));
  pnh.param<std::string>("marker_topic", g_marker_topic, std::string("/dbscan_lines"));
  pnh.param<std::string>("cloud_topic", g_cloud_topic, std::string("/scan_points"));
  pnh.param<std::string>("detected_topic", g_detected_topic, std::string("/labacorn_detected"));

  pnh.param<double>("eps", g_eps, 0.2);
  pnh.param<int>("min_samples", g_min_samples, 4);
  pnh.param<double>("range_min", g_range_min, 0.13);
  pnh.param<double>("range_max", g_range_max, 0.9);
  pnh.param<double>("front_min_deg", g_front_min_deg, 60.0);
  pnh.param<double>("front_max_deg", g_front_max_deg, 300.0);

  ROS_INFO("[labacorn_node] subscribe scan='%s'",
           ros::names::resolve(g_scan_topic).c_str());
  ROS_INFO("[labacorn_node] publish detected='%s', markers='%s', cloud='%s'",
           ros::names::resolve(g_detected_topic).c_str(),
           ros::names::resolve(g_marker_topic).c_str(),
           ros::names::resolve(g_cloud_topic).c_str());

  g_pub_marker_arr = nh.advertise<visualization_msgs::MarkerArray>(g_marker_topic, 10);
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(g_cloud_topic, 1);
  g_pub_detected = nh.advertise<std_msgs::Bool>(g_detected_topic, 10);

  ros::Subscriber scan_sub = nh.subscribe(g_scan_topic, 1, scanCallback);
  ros::spin();
  return 0;
}
