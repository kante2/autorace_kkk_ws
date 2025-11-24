// rotary_detect_lidar.cpp
// 로터리 전용: 라이다를 전처리 -> DBSCAN 클러스터링 -> /rotary_detected 퍼블리시

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

// -------------------- 파라미터/토픽 --------------------
std::string g_topic_scan;
std::string g_topic_detected;
std::string g_topic_marker;

double g_eps              = 0.3;
int    g_min_pts          = 7;
double g_max_use_dist     = 2.5;  // 전처리 시 최대 거리
double g_close_thresh     = 1.0;  // 이 거리 이내면 로터리 진입 감지
double g_scan_timeout_sec = 0.5;

// -------------------- 전역 상태 --------------------
ros::Publisher g_pub_detected;
ros::Publisher g_pub_marker;
ros::Subscriber g_sub_scan;

sensor_msgs::LaserScan::ConstPtr g_latest_scan;
ros::Time g_last_scan_time;
bool g_latest_close = false;

// -------------------- 유틸 --------------------
inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- DBSCAN/클러스터 --------------------
using PointXY = std::array<double, 2>;

struct ClusterCentroid
{
  double angle;    // deg
  double distance; // m
};

struct DetectionResult
{
  bool detected = false;
  std::vector<ClusterCentroid> centroids;
};

std::vector<PointXY> polarToCartesian(const std::vector<double>& angles_deg,
                                      const std::vector<double>& dists)
{
  std::vector<PointXY> points;
  if (angles_deg.size() != dists.size()) return points;

  points.reserve(angles_deg.size());
  for (size_t i = 0; i < angles_deg.size(); ++i) {
    double ang_rad = angles_deg[i] * M_PI / 180.0;
    points.push_back({dists[i] * std::cos(ang_rad), dists[i] * std::sin(ang_rad)});
  }
  return points;
}

std::vector<int> regionQuery(double eps, const std::vector<PointXY>& points, size_t index)
{
  std::vector<int> neighbors;
  if (index >= points.size()) return neighbors;

  for (size_t i = 0; i < points.size(); ++i) {
    double dx = points[i][0] - points[index][0];
    double dy = points[i][1] - points[index][1];
    if (std::hypot(dx, dy) <= eps) {
      neighbors.push_back(static_cast<int>(i));
    }
  }
  return neighbors;
}

std::vector<int> dbscan(int min_pts, double eps, const std::vector<PointXY>& points)
{
  if (points.empty()) return {};

  constexpr int UNCLASSIFIED = -1;
  constexpr int NOISE        = -2;

  std::vector<int> labels(points.size(), UNCLASSIFIED);
  int cluster_id = 0;

  for (size_t idx = 0; idx < points.size(); ++idx) {
    if (labels[idx] != UNCLASSIFIED) continue;

    std::vector<int> neighbors = regionQuery(eps, points, idx);
    if (static_cast<int>(neighbors.size()) < min_pts) {
      labels[idx] = NOISE;
      continue;
    }

    labels[idx] = cluster_id;
    neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), static_cast<int>(idx)),
                    neighbors.end());

    while (!neighbors.empty()) {
      int current = neighbors.back();
      neighbors.pop_back();

      if (labels[current] == NOISE) {
        labels[current] = cluster_id;
      }
      if (labels[current] != UNCLASSIFIED) {
        continue;
      }

      labels[current] = cluster_id;
      std::vector<int> current_neighbors = regionQuery(eps, points, current);
      if (static_cast<int>(current_neighbors.size()) >= min_pts) {
        for (int n : current_neighbors) {
          if (std::find(neighbors.begin(), neighbors.end(), n) == neighbors.end()) {
            neighbors.push_back(n);
          }
        }
      }
    }
    ++cluster_id;
  }
  return labels;
}

std::vector<ClusterCentroid> computeCentroids(const std::vector<PointXY>& points,
                                              const std::vector<int>& labels)
{
  std::vector<ClusterCentroid> centroids;
  if (points.size() != labels.size()) return centroids;

  std::vector<int> uniq;
  for (int l : labels) {
    if (l >= 0 && std::find(uniq.begin(), uniq.end(), l) == uniq.end()) {
      uniq.push_back(l);
    }
  }
  std::sort(uniq.begin(), uniq.end());

  for (int label : uniq) {
    double sum_x = 0.0, sum_y = 0.0;
    int count = 0;
    for (size_t i = 0; i < labels.size(); ++i) {
      if (labels[i] == label) {
        sum_x += points[i][0];
        sum_y += points[i][1];
        ++count;
      }
    }
    if (count == 0) continue;
    double cx = sum_x / count;
    double cy = sum_y / count;
    centroids.push_back({std::atan2(cy, cx) * 180.0 / M_PI, std::hypot(cx, cy)});
  }
  return centroids;
}

DetectionResult detectClusters(const std::vector<double>& angles_deg,
                               const std::vector<double>& dists,
                               double eps, int min_pts)
{
  DetectionResult result;
  if (angles_deg.empty() || dists.empty()) {
    result.detected = false;
    return result;
  }
  std::vector<PointXY> points = polarToCartesian(angles_deg, dists);
  std::vector<int> labels = dbscan(min_pts, eps, points);
  result.centroids = computeCentroids(points, labels);
  result.detected = !result.centroids.empty();
  return result;
}

// -------------------- 전처리 --------------------
double normalizeAnglePositive(double angle)
{
  const double two_pi = 2.0 * M_PI;
  double norm = std::fmod(angle, two_pi);
  if (norm < 0.0) norm += two_pi;
  return norm;
}

struct LidarProcessingResult
{
  std::vector<double> angle_deg;
  std::vector<double> dist;
};

LidarProcessingResult preprocess(const sensor_msgs::LaserScan& scan)
{
  const std::vector<float>& ranges_raw = scan.ranges;
  const size_t raw_size = ranges_raw.size();

  // 간단한 이동평균 필터
  constexpr size_t half_window = 3;
  constexpr size_t mvg_window  = 2 * half_window + 1;
  const double nan_value = std::numeric_limits<double>::quiet_NaN();

  std::vector<double> padded;
  padded.reserve(raw_size + 2 * half_window);
  padded.insert(padded.end(), half_window, nan_value);
  for (float v : ranges_raw) padded.push_back(static_cast<double>(v));
  padded.insert(padded.end(), half_window, nan_value);

  std::vector<double> filtered(raw_size, scan.range_max);
  for (size_t i = 0; i < raw_size; ++i) {
    bool contains_nan = false;
    double sum = 0.0;
    for (size_t j = 0; j < mvg_window; ++j) {
      double s = padded[i + j];
      if (std::isnan(s)) { contains_nan = true; break; }
      sum += s;
    }
    filtered[i] = contains_nan ? nan_value : sum / static_cast<double>(mvg_window);
  }

  for (double& v : filtered) {
    if (std::isnan(v) || !std::isfinite(v) || v > scan.range_max) {
      v = scan.range_max;
    }
  }

  std::vector<double> angles;
  angles.reserve(raw_size);
  double angle = scan.angle_min;
  for (size_t i = 0; i < raw_size; ++i) {
    angles.push_back(normalizeAnglePositive(angle));
    angle += scan.angle_increment;
  }

  // ROI: 120~240 deg, 거리 < g_max_use_dist
  const double lower = 120.0 / 180.0 * M_PI;
  const double upper = 240.0 / 180.0 * M_PI;

  LidarProcessingResult res;
  for (size_t i = 0; i < angles.size(); ++i) {
    double ang = angles[i];
    if (ang < lower || ang > upper) continue;
    if (filtered[i] < g_max_use_dist) {
      res.angle_deg.push_back(ang * 180.0 / M_PI);
      res.dist.push_back(filtered[i]);
    }
  }

  return res;
}

// -------------------- 마커 퍼블리시 --------------------
void publishMarkers(const std::vector<ClusterCentroid>& centroids)
{
  if (g_pub_marker.getNumSubscribers() == 0) return;

  visualization_msgs::MarkerArray arr;
  ros::Time stamp = ros::Time::now();

  for (size_t i = 0; i < centroids.size(); ++i) {
    visualization_msgs::Marker m;
    m.header.frame_id = "laser";
    m.header.stamp    = stamp;
    m.ns   = "rotary_objects";
    m.id   = static_cast<int>(i);
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;

    double ang = centroids[i].angle * M_PI / 180.0;
    double d   = centroids[i].distance;
    m.pose.position.x = d * std::cos(ang);
    m.pose.position.y = d * std::sin(ang);
    m.pose.position.z = 0.0;
    m.pose.orientation.w = 1.0;

    m.scale.x = 0.2;
    m.scale.y = 0.2;
    m.scale.z = 0.2;

    m.color.r = 0.2f;
    m.color.g = 0.6f;
    m.color.b = 1.0f;
    m.color.a = 0.8f;

    arr.markers.push_back(m);
  }

  g_pub_marker.publish(arr);
}

// -------------------- 콜백 --------------------
void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  g_latest_scan = msg;
  g_last_scan_time = ros::Time::now();

  const LidarProcessingResult processed = preprocess(*msg);
  const DetectionResult detection = detectClusters(processed.angle_deg, processed.dist, g_eps, g_min_pts);

  g_latest_close = false;
  if (detection.detected) {
    for (const auto& c : detection.centroids) {
      if (c.distance <= g_close_thresh) {
        g_latest_close = true;
      }
    }
    publishMarkers(detection.centroids);
  }

  std_msgs::Bool detected_msg;
  detected_msg.data = g_latest_close;
  g_pub_detected.publish(detected_msg);

  ROS_INFO_THROTTLE(1.0,
    "[rotary_detect_lidar] clusters=%zu close=%d",
    detection.centroids.size(), static_cast<int>(g_latest_close));
}

// -------------------- main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rotary_detect_lidar");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<std::string>("scan_topic",    g_topic_scan,    std::string("/scan"));
  pnh.param<std::string>("detected_topic", g_topic_detected, std::string("/rotary_detected"));
  pnh.param<std::string>("marker_topic",  g_topic_marker,  std::string("rotary/obstacle_markers"));
  pnh.param<double>("eps",               g_eps,              0.3);
  pnh.param<int>("min_pts",              g_min_pts,          7);
  pnh.param<double>("max_use_dist",      g_max_use_dist,     2.5);
  pnh.param<double>("close_thresh",      g_close_thresh,     1.0);
  pnh.param<double>("scan_timeout_sec",  g_scan_timeout_sec, 0.5);

  ROS_INFO("[rotary_detect_lidar] subscribe scan='%s'", ros::names::resolve(g_topic_scan).c_str());
  ROS_INFO("[rotary_detect_lidar] publish detected='%s', markers='%s'",
           ros::names::resolve(g_topic_detected).c_str(),
           ros::names::resolve(g_topic_marker).c_str());

  g_pub_detected = nh.advertise<std_msgs::Bool>(g_topic_detected, 1);
  g_pub_marker   = nh.advertise<visualization_msgs::MarkerArray>(g_topic_marker, 1);
  g_sub_scan     = nh.subscribe(g_topic_scan, 1, scanCb);

  g_last_scan_time = ros::Time(0);
  g_latest_close = false;

  ros::spin();
  return 0;
}
