#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <laser_geometry/laser_geometry.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

// -------------------- 2D 포인트 --------------------
struct Point2D {
  double x;
  double y;
};

// -------------------- 전역 파라미터/토픽 --------------------
std::string g_scan_topic;
std::string g_marker_topic;
std::string g_cloud_topic;
std::string g_rubbercone_detected_topic;

double g_eps = 0.2;
int    g_min_samples = 3;
double g_range_min = 0.13;
double g_range_max = 1.5;
double g_front_min_deg = 60.0;
double g_front_max_deg = 300.0;
double g_detect_distance = 0.5;  // [m] 이내면 감지 true

// -------------------- 전역 Pub --------------------
ros::Publisher g_pub_marker;
ros::Publisher g_pub_cloud;
ros::Publisher g_pub_rubber_detected;
laser_geometry::LaserProjection g_projector;

// -------------------- DBSCAN --------------------
std::vector<int> dbscan(const std::vector<Point2D> &points) {
  const int n = static_cast<int>(points.size());
  std::vector<int> labels(n, -1);
  if (n == 0) return labels;

  int cluster_id = 0;
  auto dist = [](const Point2D &a, const Point2D &b) { return std::hypot(a.x - b.x, a.y - b.y); };

  for (int i = 0; i < n; ++i) {
    if (labels[i] != -1) continue;

    std::vector<int> neighbors;
    for (int j = 0; j < n; ++j) {
      if (dist(points[i], points[j]) <= g_eps) {
        neighbors.push_back(j);
      }
    }

    if (static_cast<int>(neighbors.size()) < g_min_samples) continue;  // 노이즈

    ++cluster_id;
    labels[i] = cluster_id;
    std::vector<int> seed_set = neighbors;
    for (std::size_t k = 0; k < seed_set.size(); ++k) {
      int j = seed_set[k];
      if (labels[j] != -1) continue;
      labels[j] = cluster_id;

      std::vector<int> j_neighbors;
      for (int m = 0; m < n; ++m) {
        if (dist(points[j], points[m]) <= g_eps) {
          j_neighbors.push_back(m);
        }
      }
      if (static_cast<int>(j_neighbors.size()) >= g_min_samples) {
        seed_set.insert(seed_set.end(), j_neighbors.begin(), j_neighbors.end());
      }
    }
  }
  return labels;
}

// -------------------- 스캔 필터링 --------------------
std::vector<Point2D> filterScanPoints(const sensor_msgs::LaserScan::ConstPtr &scan) {
  std::vector<Point2D> points;
  points.reserve(scan->ranges.size());

  double angle = scan->angle_min;
  for (float r : scan->ranges) {
    if (!std::isfinite(r) || r < g_range_min || r > g_range_max) {
      angle += scan->angle_increment;
      continue;
    }

    double angle_deg = angle * 180.0 / M_PI;
    if (angle_deg < 0.0) angle_deg += 360.0;
    if (angle_deg < g_front_min_deg || angle_deg > g_front_max_deg) {
      angle += scan->angle_increment;
      continue;
    }

    Point2D p{r * std::cos(angle), r * std::sin(angle)};
    points.push_back(p);
    angle += scan->angle_increment;
  }
  return points;
}

// -------------------- 감지 + 시각화 --------------------
bool detectAndVisualize(const std::vector<Point2D> &points,
                        const std_msgs::Header &header) {
  if (points.empty()) return false;

  const std::vector<int> labels = dbscan(points);
  if (labels.empty()) return false;
  const int max_label = *std::max_element(labels.begin(), labels.end());
  if (max_label < 1) return false;  // 전부 노이즈

  bool detected = false;
  int marker_id = 0;

  for (int c = 1; c <= max_label; ++c) {
    double cx = 0.0, cy = 0.0;
    int count = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (labels[i] == c) {
        cx += points[i].x;
        cy += points[i].y;
        ++count;
      }
    }
    if (count == 0) continue;
    cx /= count;
    cy /= count;

    if (std::hypot(cx, cy) <= g_detect_distance) {
      detected = true;
    }

    if (g_pub_marker.getNumSubscribers() > 0) {
      visualization_msgs::Marker marker;
      marker.header = header;
      marker.ns = "rubbercone_clusters";
      marker.id = marker_id++;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = cx;
      marker.pose.position.y = cy;
      marker.pose.position.z = 0.0;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.15;
      marker.color.r = 1.0;
      marker.color.g = 0.5;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration(0.2);
      g_pub_marker.publish(marker);
    }
  }
  return detected;
}

// -------------------- 스캔 콜백 --------------------
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
  if (g_pub_cloud.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 cloud;
    g_projector.projectLaser(*scan, cloud);
    cloud.header = scan->header;
    g_pub_cloud.publish(cloud);
  }

  const std::vector<Point2D> points = filterScanPoints(scan);
  const bool detected = detectAndVisualize(points, scan->header);

  std_msgs::Bool msg;
  msg.data = detected;
  g_pub_rubber_detected.publish(msg);

  ROS_INFO_THROTTLE(0.5,
                    "[rubbercone_node] points=%zu detected=%d (dist<=%.2f m)",
                    points.size(),
                    static_cast<int>(detected),
                    g_detect_distance);
}

// -------------------- main --------------------
int main(int argc, char **argv) {
  ros::init(argc, argv, "rubbercone_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<std::string>("scan_topic", g_scan_topic, std::string("/scan"));
  pnh.param<std::string>("marker_topic", g_marker_topic, std::string("/rubbercone_clusters"));
  pnh.param<std::string>("cloud_topic", g_cloud_topic, std::string("/rubbercone_cloud"));
  pnh.param<std::string>("rubbercone_detected_topic",
                         g_rubbercone_detected_topic,
                         std::string("/rubbercone_detected"));

  pnh.param<double>("eps", g_eps, 0.2);
  pnh.param<int>("min_samples", g_min_samples, 3);
  pnh.param<double>("range_min", g_range_min, 0.13);
  pnh.param<double>("range_max", g_range_max, 1.5);
  pnh.param<double>("front_min_deg", g_front_min_deg, 60.0);
  pnh.param<double>("front_max_deg", g_front_max_deg, 300.0);
  pnh.param<double>("detect_distance", g_detect_distance, 0.5);

  ROS_INFO("[rubbercone_node] subscribe scan='%s'",
           ros::names::resolve(g_scan_topic).c_str());
  ROS_INFO("[rubbercone_node] publish rubbercone='%s', marker='%s', cloud='%s'",
           ros::names::resolve(g_rubbercone_detected_topic).c_str(),
           ros::names::resolve(g_marker_topic).c_str(),
           ros::names::resolve(g_cloud_topic).c_str());

  g_pub_marker = nh.advertise<visualization_msgs::Marker>(g_marker_topic, 10);
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(g_cloud_topic, 1);
  g_pub_rubber_detected = nh.advertise<std_msgs::Bool>(g_rubbercone_detected_topic, 10);

  ros::Subscriber sub_scan = nh.subscribe(g_scan_topic, 1, scanCallback);
  ros::spin();
  return 0;
}
