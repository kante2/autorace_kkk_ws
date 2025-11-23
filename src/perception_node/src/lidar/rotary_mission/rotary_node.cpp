#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#include <string>
#include <vector>

#include "clustering_visualization.hpp"
#include "lidar_preprocessing_rotary.hpp"
#include "obstacle_detect.hpp"

// -------------------- 전역 --------------------
std::string g_scan_topic;
std::string g_detected_topic;
std::string g_marker_topic;

double g_eps = 0.3;
int g_min_pts = 7;
double g_close_dist = 1.0;  // 이 거리 이내 클러스터 존재 시 감지 true

ros::Subscriber g_sub_scan;
ros::Publisher g_pub_detected;
std::unique_ptr<ClusterVisualizer> g_visualizer;

// -------------------- 콜백 --------------------
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  const LidarProcessingResult pre = preprocessLidar(*scan_msg);
  const DetectionResult det = detect(pre.angle_ranges_deg, pre.dist_ranges, g_eps, g_min_pts);

  bool has_close = false;
  if (det.detected)
  {
    for (const auto &c : det.centroids)
    {
      if (c.distance <= g_close_dist)
      {
        has_close = true;
        break;
      }
    }
  }

  std_msgs::Bool msg;
  msg.data = has_close;
  g_pub_detected.publish(msg);

  if (g_visualizer && det.detected)
  {
    g_visualizer->publish(det);
  }

  ROS_INFO_THROTTLE(1.0, "[rotary_node] detected=%d clusters=%zu",
                    static_cast<int>(has_close), det.centroids.size());
}

// -------------------- main --------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotary_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<std::string>("scan_topic", g_scan_topic, std::string("/scan"));
  pnh.param<std::string>("detected_topic", g_detected_topic, std::string("/rotary_detected"));
  pnh.param<std::string>("marker_topic", g_marker_topic, std::string("rotary/obstacle_markers"));

  pnh.param<double>("eps", g_eps, 0.3);
  pnh.param<int>("min_pts", g_min_pts, 7);
  pnh.param<double>("close_dist", g_close_dist, 1.0);

  ROS_INFO("[rotary_node] subscribe scan='%s'", ros::names::resolve(g_scan_topic).c_str());
  ROS_INFO("[rotary_node] publish detected='%s', marker='%s'",
           ros::names::resolve(g_detected_topic).c_str(),
           ros::names::resolve(g_marker_topic).c_str());

  g_pub_detected = nh.advertise<std_msgs::Bool>(g_detected_topic, 1);
  g_visualizer = std::make_unique<ClusterVisualizer>(g_marker_topic);

  g_sub_scan = nh.subscribe(g_scan_topic, 1, scanCallback);

  ros::spin();
  return 0;
}
