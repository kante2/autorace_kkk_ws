#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <laser_geometry/laser_geometry.h>

#include "gate_types.hpp"
#include "lidar_preprocessing_gate.hpp"

// -------------------- DBSCAN --------------------
static std::vector<int> dbscan(const std::vector<Point2D>& points, double eps, int min_samples)
{
  const int n = static_cast<int>(points.size());
  std::vector<int> labels(n, -1);
  if (n == 0)
    return labels;

  using PointT = pcl::PointXYZ;

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  cloud->points.reserve(n);

  for (const auto& p : points)
  {
    PointT pt;
    pt.x = static_cast<float>(p.x);
    pt.y = static_cast<float>(p.y);
    pt.z = 0.0f;
    cloud->points.push_back(pt);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;

  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cloud);

  std::vector<int> neighbor_indices;
  std::vector<float> neighbor_sq_dists;

  int cluster_id = 0;

  for (int i = 0; i < n; ++i)
  {
    if (labels[i] != -1)
      continue;

    PointT searchPoint = cloud->points[i];

    neighbor_indices.clear();
    neighbor_sq_dists.clear();

    int found = kdtree.radiusSearch(searchPoint,
                                    static_cast<float>(eps),
                                    neighbor_indices,
                                    neighbor_sq_dists,
                                    0);

    if (found < min_samples)
      continue;

    ++cluster_id;
    labels[i] = cluster_id;

    std::vector<int> seed_set;
    seed_set.reserve(found);
    for (int idx : neighbor_indices)
    {
      if (idx == i)
        continue;
      seed_set.push_back(idx);
    }

    for (size_t k = 0; k < seed_set.size(); ++k)
    {
      int j = seed_set[k];
      if (labels[j] != -1)
        continue;

      labels[j] = cluster_id;

      searchPoint = cloud->points[j];
      neighbor_indices.clear();
      neighbor_sq_dists.clear();

      int found_j = kdtree.radiusSearch(searchPoint,
                                        static_cast<float>(eps),
                                        neighbor_indices,
                                        neighbor_sq_dists,
                                        0);

      if (found_j >= min_samples)
      {
        for (int m : neighbor_indices)
        {
          if (labels[m] == -1)
            seed_set.push_back(m);
        }
      }
    }
  }

  return labels;
}

// -------------------- 클러스터 중심 --------------------
static bool computeCenter(const std::vector<Point2D>& cluster, double& mx, double& my)
{
  if (cluster.empty())
  {
    mx = my = 0.0;
    return false;
  }

  mx = 0.0;
  my = 0.0;
  for (const auto& p : cluster)
  {
    mx += p.x;
    my += p.y;
  }
  mx /= cluster.size();
  my /= cluster.size();
  return true;
}

static bool computeGateCluster(const std::vector<Point2D>& points,
                               const std::vector<int>& labels,
                               std::vector<Point2D>& gate_cluster,
                               double& mx,
                               double& my,
                               int min_samples)
{
  gate_cluster.clear();
  mx = my = 0.0;

  if (points.empty() || labels.empty())
    return false;

  int max_label = *std::max_element(labels.begin(), labels.end());
  if (max_label <= 0)
    return false;

  std::vector<std::vector<Point2D>> clusters(max_label + 1);
  for (size_t i = 0; i < points.size(); ++i)
  {
    int cid = labels[i];
    if (cid <= 0)
      continue;
    clusters[cid].push_back(points[i]);
  }

  int best_id = -1;
  double best_x = std::numeric_limits<double>::max();

  for (int cid = 1; cid <= max_label; ++cid)
  {
    const auto& c = clusters[cid];
    if (static_cast<int>(c.size()) < min_samples)
      continue;

    double cx, cy;
    if (!computeCenter(c, cx, cy))
      continue;

    if (cx < best_x)
    {
      best_x = cx;
      best_id = cid;
    }
  }

  if (best_id < 0)
    return false;

  gate_cluster = clusters[best_id];
  computeCenter(gate_cluster, mx, my);
  return true;
}

static double computeDistance(double x, double y)
{
  return std::sqrt(x * x + y * y);
}

static GateState computeGateState(const std::vector<Point2D>& gate_cluster, double ratio_threshold)
{
  if (gate_cluster.empty())
    return GATE_UP;

  double mx, my;
  if (!computeCenter(gate_cluster, mx, my))
    return GATE_UP;

  double var_x = 0.0;
  double var_y = 0.0;
  for (const auto& p : gate_cluster)
  {
    double dx = p.x - mx;
    double dy = p.y - my;
    var_x += dx * dx;
    var_y += dy * dy;
  }
  var_x /= gate_cluster.size();
  var_y /= gate_cluster.size();

  if (var_y > ratio_threshold * var_x)
    return GATE_DOWN;
  return GATE_UP;
}

static bool applyHysteresis(GateState gate_state,
                            int gate_down_thresh,
                            int gate_up_thresh,
                            int& gate_down_count,
                            int& gate_up_count)
{
  bool gate_down_final = false;

  if (gate_state == GATE_DOWN)
  {
    ++gate_down_count;
    gate_up_count = 0;
  }
  else
  {
    ++gate_up_count;
    gate_down_count = 0;
  }

  if (gate_down_count >= gate_down_thresh)
    gate_down_final = true;
  if (gate_up_count >= gate_up_thresh)
    gate_down_final = false;

  return gate_down_final;
}

GateDetectionResult detectGate(const std::vector<Point2D>& points,
                               double eps,
                               int min_samples,
                               int gate_down_thresh,
                               int gate_up_thresh,
                               int& gate_down_count,
                               int& gate_up_count)
{
  GateDetectionResult result;

  std::vector<int> labels = dbscan(points, eps, min_samples);
  if (!labels.empty())
    result.cluster_count = *std::max_element(labels.begin(), labels.end());

  std::vector<Point2D> gate_cluster;
  double gate_cx = 0.0;
  double gate_cy = 0.0;

  const bool has_gate_cluster = computeGateCluster(points,
                                                   labels,
                                                   gate_cluster,
                                                   gate_cx,
                                                   gate_cy,
                                                   min_samples);

  if (has_gate_cluster)
  {
    result.has_gate = true;
    result.gate_cx = gate_cx;
    result.gate_cy = gate_cy;
    result.gate_dist = computeDistance(gate_cx, gate_cy);
    result.gate_state = computeGateState(gate_cluster, 2.0);
    result.gate_cluster = gate_cluster;
  }

  const bool gate_down_final =
      applyHysteresis(result.gate_state, gate_down_thresh, gate_up_thresh, gate_down_count, gate_up_count);

  result.gate_down_final = gate_down_final;

  if (!result.has_gate)
    result.gate_dist = std::numeric_limits<double>::infinity();

  if (gate_down_final)
    result.gate_state = GATE_DOWN;
  else
    result.gate_state = GATE_UP;

  return result;
}

// -------------------- 노드 전역 --------------------
std::string g_scan_topic;
std::string g_gate_detected_topic;
std::string g_gate_distance_topic;
std::string g_marker_topic;
std::string g_cloud_topic;

double g_eps = 0.25;
int    g_min_samples = 6;
double g_range_min = 0.13;
double g_range_max = 2.0;
double g_front_min_deg = 60.0;
double g_front_max_deg = 300.0;
int g_gate_down_thresh = 3;
int g_gate_up_thresh = 3;

ros::Subscriber g_scan_sub;
ros::Publisher  g_pub_detected;
ros::Publisher  g_pub_distance;
ros::Publisher  g_pub_marker;
ros::Publisher  g_pub_cloud;

laser_geometry::LaserProjection g_projector;

int g_gate_down_count = 0;
int g_gate_up_count   = 0;

// -------------------- 퍼블리시 --------------------
void publishMarker(const std::vector<Point2D>& cluster, const std::string& frame_id)
{
  visualization_msgs::Marker mk;
  mk.header.stamp = ros::Time::now();
  mk.header.frame_id = frame_id;
  mk.ns = "gate_cluster";
  mk.id = 0;
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::ADD;
  mk.scale.x = 0.05;
  mk.scale.y = 0.05;
  mk.scale.z = 0.05;
  mk.color.r = 0.0f;
  mk.color.g = 1.0f;
  mk.color.b = 0.0f;
  mk.color.a = 1.0f;

  for (const auto& p : cluster)
  {
    geometry_msgs::Point gp;
    gp.x = p.x;
    gp.y = p.y;
    gp.z = 0.0;
    mk.points.push_back(gp);
  }
  g_pub_marker.publish(mk);
}

// -------------------- 콜백 --------------------
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  GateLidarPreprocessResult prep = preprocessGateLidar(*scan_msg,
                                                       g_range_min,
                                                       g_range_max,
                                                       g_front_min_deg,
                                                       g_front_max_deg,
                                                       g_projector,
                                                       true);

  if (prep.has_cloud)
  {
    g_pub_cloud.publish(prep.cloud);
  }

  GateDetectionResult res = detectGate(prep.points,
                                       g_eps,
                                       g_min_samples,
                                       g_gate_down_thresh,
                                       g_gate_up_thresh,
                                       g_gate_down_count,
                                       g_gate_up_count);

  std_msgs::Bool det_msg;
  det_msg.data = res.gate_down_final;
  g_pub_detected.publish(det_msg);

  std_msgs::Float64 dist_msg;
  dist_msg.data = res.gate_dist;
  g_pub_distance.publish(dist_msg);

  if (!res.gate_cluster.empty())
  {
    publishMarker(res.gate_cluster, scan_msg->header.frame_id);
  }

  ROS_INFO_THROTTLE(1.0, "[gate_node] detected=%d dist=%.2f clusters=%d",
                    static_cast<int>(res.gate_down_final),
                    res.gate_dist,
                    res.cluster_count);
}

// -------------------- main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "gate_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<std::string>("scan_topic", g_scan_topic, std::string("/scan"));
  pnh.param<std::string>("gate_detected_topic", g_gate_detected_topic, std::string("/gate_detected"));
  pnh.param<std::string>("gate_distance_topic", g_gate_distance_topic, std::string("/gate_distance"));
  pnh.param<std::string>("marker_topic", g_marker_topic, std::string("/gate_cluster_marker"));
  pnh.param<std::string>("cloud_topic", g_cloud_topic, std::string("/gate_scan_cloud"));

  pnh.param<double>("eps", g_eps, 0.25);
  pnh.param<int>("min_samples", g_min_samples, 6);
  pnh.param<double>("range_min", g_range_min, 0.13);
  pnh.param<double>("range_max", g_range_max, 2.0);
  pnh.param<double>("front_min_deg", g_front_min_deg, 60.0);
  pnh.param<double>("front_max_deg", g_front_max_deg, 300.0);
  pnh.param<int>("gate_down_thresh", g_gate_down_thresh, 3);
  pnh.param<int>("gate_up_thresh", g_gate_up_thresh, 3);

  ROS_INFO("[gate_node] subscribe scan='%s'", ros::names::resolve(g_scan_topic).c_str());
  ROS_INFO("[gate_node] publish detected='%s', distance='%s', marker='%s', cloud='%s'",
           ros::names::resolve(g_gate_detected_topic).c_str(),
           ros::names::resolve(g_gate_distance_topic).c_str(),
           ros::names::resolve(g_marker_topic).c_str(),
           ros::names::resolve(g_cloud_topic).c_str());

  g_pub_detected = nh.advertise<std_msgs::Bool>(g_gate_detected_topic, 1);
  g_pub_distance = nh.advertise<std_msgs::Float64>(g_gate_distance_topic, 1);
  g_pub_marker   = nh.advertise<visualization_msgs::Marker>(g_marker_topic, 1);
  g_pub_cloud    = nh.advertise<sensor_msgs::PointCloud2>(g_cloud_topic, 1);
  g_scan_sub     = nh.subscribe(g_scan_topic, 1, scanCallback);

  ros::spin();
  return 0;
}
