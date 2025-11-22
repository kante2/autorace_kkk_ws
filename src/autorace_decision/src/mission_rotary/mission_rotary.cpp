// mission_rotary.cpp
// rotary_main.cpp의 로직을 함수화하여 main_node와 디버그용 main에서 재사용

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#include <string>

#include "clustering_visualization.hpp"
#include "lidar_preprocessing_rotary.cpp"
#include "object_detect.hpp"

namespace {
// 토픽 이름
std::string g_topic_scan;
std::string g_topic_rotary_objects;
std::string g_topic_marker;

// DBSCAN 파라미터
double g_eps = 0.3;
int g_min_pts = 7;

// 상태
ros::Publisher g_cluster_pub;
ros::Publisher g_rotary_objects_pub;
ros::Subscriber g_scan_sub;
sensor_msgs::LaserScan::ConstPtr g_scan_msg;

// -------------------- 콜백 --------------------
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) { g_scan_msg = scan_msg; }

// -------------------- 내부 처리 --------------------
void processOnce() {
  if (!g_scan_msg) {
    return;
  }

  const LidarProcessingResult processed = preprocessLidar(*g_scan_msg);
  const DetectionResult detection = detect(processed.angle_ranges_deg, processed.dist_ranges, g_eps, g_min_pts);

  bool has_close_object = false;
  if (detection.detected) {
    for (std::size_t cluster_id = 0; cluster_id < detection.centroids.size(); ++cluster_id) {
      const ClusterCentroid &center = detection.centroids[cluster_id];
      ROS_INFO("id=%zu angle(deg)=%.3f distance(m)=%.3f", cluster_id, center.angle, center.distance);
      if (center.distance <= 1.0) {
        has_close_object = true;
      }
    }
    publishClusterMarkers(g_cluster_pub, detection.centroids);
  } else {
    ROS_INFO("not detected");
  }

  std_msgs::Bool msg;
  msg.data = has_close_object;
  g_rotary_objects_pub.publish(msg);
}
}  // namespace

// =====================================================
// main_node.cpp 에서 사용할 init / step 함수
// =====================================================

void mission_rotary_init(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  ROS_INFO("[mission_rotary] init");

  // 파라미터 로드
  pnh.param<std::string>("scan_topic", g_topic_scan, std::string("/scan"));
  pnh.param<std::string>("rotary_objects_topic", g_topic_rotary_objects, std::string("/rotary_detected"));
  pnh.param<std::string>("marker_topic", g_topic_marker, std::string("rotary/obstacle_markers"));
  pnh.param<double>("eps", g_eps, 0.3);
  pnh.param<int>("min_pts", g_min_pts, 7);

  ROS_INFO("[mission_rotary] subscribe scan='%s'",
           ros::names::resolve(g_topic_scan).c_str());
  ROS_INFO("[mission_rotary] publish rotary_objects='%s', markers='%s'",
           ros::names::resolve(g_topic_rotary_objects).c_str(),
           ros::names::resolve(g_topic_marker).c_str());

  // Pub/Sub 설정
  g_cluster_pub = initClusterVisualizer(nh, g_topic_marker);
  g_rotary_objects_pub = nh.advertise<std_msgs::Bool>(g_topic_rotary_objects, 1);
  g_scan_sub = nh.subscribe(g_topic_scan, 1, scanCallback);

  g_scan_msg.reset();

  ROS_INFO("[mission_rotary] init done");
}

// main loop에서 호출
void mission_rotary_step() {
  processOnce();
}
