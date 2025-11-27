#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
std::string g_target_topic;
std::string g_target_frame;

double g_eps = 0.2;
int    g_min_samples = 3;
double g_range_min = 0.13;
double g_range_max = 0.9;
double g_front_min_deg = 60.0;
double g_front_max_deg = 300.0;
double g_lane_offset_y = 0.12;
double g_lane_gain = 1.8;

// -------------------- 전역 Pub --------------------
ros::Publisher g_pub_marker_arr;
ros::Publisher g_pub_cloud;
ros::Publisher g_pub_detected;
ros::Publisher g_pub_target;
laser_geometry::LaserProjection g_projector;
std::shared_ptr<tf2_ros::Buffer> g_tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> g_tf_listener;

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

// -------------------- 타깃 계산 --------------------
bool computeTarget(const std::vector<Point2D>& centroids, double lane_offset_y,
                   double& target_x, double& target_y)
{
  double left_x_sum = 0.0, left_y_sum = 0.0;
  double right_x_sum = 0.0, right_y_sum = 0.0;
  int left_cnt = 0, right_cnt = 0;

  for (const auto& p : centroids)
  {
    if (p.y > 0.0)
    {
      left_x_sum += p.x;
      left_y_sum += p.y;
      ++left_cnt;
    }
    else
    {
      right_x_sum += p.x;
      right_y_sum += p.y;
      ++right_cnt;
    }
  }

  if (left_cnt > 0 && right_cnt > 0)
  {
    double lx = left_x_sum / left_cnt;
    double ly = left_y_sum / left_cnt;
    double rx = right_x_sum / right_cnt;
    double ry = right_y_sum / right_cnt;
    int total = left_cnt + right_cnt;
    target_x = static_cast<double>(right_cnt) / total * lx +
               static_cast<double>(left_cnt) / total * rx;
    target_y = static_cast<double>(right_cnt) / total * ly +
               static_cast<double>(left_cnt) / total * ry;
    return true;
  }
  else if (left_cnt > 0)
  {
    double rx = right_x_sum / right_cnt;
    double ry = right_y_sum / right_cnt;

    // 오른쪽 클러스터 기준으로 중앙선 계산
    // 장애물 폭 0.44m 기준, 차량이 너무 왼쪽으로 튀지 않게
    target_x = rx;
    target_y = ry - lane_offset_y * g_lane_gain;  // 중앙선 = 오른쪽 y - 반 폭
    //target_x = left_x_sum / left_cnt;
    //target_y = (left_y_sum / left_cnt) - lane_offset_y;
    // target_y = (left_y_sum / left_cnt) - lane_offset_y;

    return true;
  }
  else if (right_cnt > 0)
  {

    double rx = right_x_sum / right_cnt;
    double ry = right_y_sum / right_cnt;  

    target_x = rx;
    target_y = ry + lane_offset_y * g_lane_gain;

    //target_x = right_x_sum / right_cnt;
    //target_y = (right_y_sum / right_cnt)+ lane_offset_y;
    //target_y = (right_y_sum / right_cnt) + lane_offset_y;
    return true;
  }
  return false;
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

  // 클러스터 수 제한: 총 2개 이하, 섹터별 1개씩 (120~170도, 190~240도)
  // 크기와 무관하게 각 섹터 첫 번째 클러스터만 사용
  {
    std::vector<Point2D> sector1, sector2;
    for (const auto& c : centroids) {
      double ang = std::atan2(c.y, c.x) * 180.0 / M_PI; // laser 프레임 기준
      if (ang < 0.0) ang += 360.0;
      if (ang >= 120.0 && ang <= 165.0) {
        if (sector1.empty()) sector1.push_back(c);
      } else if (ang >= 215.0 && ang <= 240.0) {
        if (sector2.empty()) sector2.push_back(c);
      }
    }

    std::vector<Point2D> limited;
    if (!sector1.empty()) limited.push_back(sector1.front());
    if (!sector2.empty()) limited.push_back(sector2.front());
    // 섹터에 아무 것도 없으면 기존 centroids를 유지해 감지 자체가 사라지지 않도록 함
    if (!limited.empty()) centroids = limited;
  }
  bool detected = !centroids.empty();

  // 2-1) 타깃 계산 및 퍼블리시
  double target_x = 0.0, target_y = 0.0;
  bool have_target = computeTarget(centroids, g_lane_offset_y, target_x, target_y);
  geometry_msgs::PointStamped pt_out;  // 퍼블리시용 변환된 타깃
  if (have_target)
  {
    // 시각화용(레이저 프레임) 좌표는 그대로 사용, 퍼블리시용은 부호 반전 후 base_link 등으로 변환
    double pub_x = -target_x;
    double pub_y = -target_y;

    geometry_msgs::PointStamped pt_pub;
    pt_pub.header = scan->header;  // laser 프레임
    pt_pub.point.x = pub_x;
    pt_pub.point.y = pub_y;
    pt_pub.point.z = 0.0;
    pt_out = pt_pub;

    // frame 변환 (laser -> target_frame) 시도
    if (!g_target_frame.empty() && pt_pub.header.frame_id != g_target_frame && g_tf_buffer)
    {
      try {
        pt_out = g_tf_buffer->transform(pt_pub, g_target_frame, ros::Duration(0.05));
      } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "[labacorn_node] TF transform failed (%s -> %s): %s",
                          pt_pub.header.frame_id.c_str(), g_target_frame.c_str(), ex.what());
        pt_out = pt_pub;  // fallback: original frame
      }
    }
    g_pub_target.publish(pt_out);

    ROS_INFO_THROTTLE(0.5, "[labacorn_node] target_pub=(%.2f, %.2f) frame=%s",
                      pt_out.point.x, pt_out.point.y, pt_out.header.frame_id.c_str());
  }

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
    if (have_target)
    {
      visualization_msgs::Marker mt;
      mt.header = scan->header;  // 시각화는 laser 프레임 그대로
      mt.ns = "dbscan_target";
      mt.id = 0;
      mt.type = visualization_msgs::Marker::SPHERE;
      mt.action = visualization_msgs::Marker::ADD;
      mt.pose.position.x = target_x;
      mt.pose.position.y = target_y;
      mt.pose.position.z = 0.0;
      mt.scale.x = mt.scale.y = mt.scale.z = 0.15;
      mt.color.r = 0.0f;
      mt.color.g = 0.6f;
      mt.color.b = 1.0f;
      mt.color.a = 1.0f;
      mt.lifetime = ros::Duration(0.2);
      ma.markers.push_back(mt);
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
  pnh.param<std::string>("target_topic", g_target_topic, std::string("/labacorn/target"));
  pnh.param<std::string>("target_frame", g_target_frame, std::string("laser"));

  pnh.param<double>("eps", g_eps, 0.2);
  pnh.param<int>("min_samples", g_min_samples, 3);
  pnh.param<double>("range_min", g_range_min, 0.3);
  pnh.param<double>("range_max", g_range_max, 0.65);
  pnh.param<double>("front_min_deg", g_front_min_deg, 120.0);
  pnh.param<double>("front_max_deg", g_front_max_deg, 240.0);
  pnh.param<double>("lane_offset_y", g_lane_offset_y, 0.25);

  ROS_INFO("[labacorn_node] subscribe scan='%s'",
           ros::names::resolve(g_scan_topic).c_str());
  ROS_INFO("[labacorn_node] publish detected='%s', markers='%s', cloud='%s', target='%s'",
           ros::names::resolve(g_detected_topic).c_str(),
           ros::names::resolve(g_marker_topic).c_str(),
           ros::names::resolve(g_cloud_topic).c_str(),
           ros::names::resolve(g_target_topic).c_str());

  g_pub_marker_arr = nh.advertise<visualization_msgs::MarkerArray>(g_marker_topic, 10);
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(g_cloud_topic, 1);
  g_pub_detected = nh.advertise<std_msgs::Bool>(g_detected_topic, 10);
  g_pub_target = nh.advertise<geometry_msgs::PointStamped>(g_target_topic, 10);

  // TF buffer/listener
  g_tf_buffer = std::make_shared<tf2_ros::Buffer>();
  g_tf_listener = std::make_shared<tf2_ros::TransformListener>(*g_tf_buffer);

  ros::Subscriber scan_sub = nh.subscribe(g_scan_topic, 1, scanCallback);
  ros::spin();
  return 0;
}