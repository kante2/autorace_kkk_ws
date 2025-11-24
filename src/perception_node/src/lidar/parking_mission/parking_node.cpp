#include "parking_lot_common.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using parking_lot::GoalResult;
using parking_lot::computeGoalFromLines;
using parking_lot::parkingDetect;
using parking_lot::preprocessLidar;

// -------------------- 전역 --------------------
std::string g_scan_topic;
std::string g_detected_topic;
std::string g_goal_topic;
std::string g_goal_frame_id;
std::string g_goal_marker_topic;

// 검출 파라미터
int    g_ransac_max_lines   = 5;
int    g_ransac_max_iters   = 200;
double g_ransac_dist_thresh = 0.05;
int    g_ransac_min_inliers = 30;
double g_parallel_angle_deg = 15.0;
double g_orth_angle_deg     = 15.0;

// 슬롯 크기 파라미터
double g_min_width   = 0.3;
double g_min_depth   = 0.2;
double g_wall_offset = 0.1;

// 상태
ros::Subscriber g_sub_scan;
ros::Publisher  g_pub_detected;
ros::Publisher  g_pub_goal;
ros::Publisher  g_pub_goal_marker;
sensor_msgs::LaserScan::ConstPtr g_last_scan;
std::optional<geometry_msgs::PoseStamped> g_last_goal;

// -------------------- 콜백 --------------------
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  g_last_scan = msg;
}

// -------------------- 메인 루프 처리 --------------------
void process()
{
  std_msgs::Bool detected_msg;
  detected_msg.data = false;

  // goal 캐시 재전송
  if (g_last_goal.has_value())
  {
    g_pub_goal.publish(*g_last_goal);
    if (g_pub_goal_marker)
    {
      visualization_msgs::Marker marker;
      marker.header = g_last_goal->header;
      marker.ns = "parking_goal";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = g_last_goal->pose;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.r = 0.2f;
      marker.color.g = 0.8f;
      marker.color.b = 0.2f;
      marker.color.a = 0.9f;
      g_pub_goal_marker.publish(marker);
    }
  }

  if (!g_last_scan)
  {
    g_pub_detected.publish(detected_msg);
    return;
  }

  std::vector<double> angles_deg;
  std::vector<double> dists;
  if (!preprocessLidar(*g_last_scan, angles_deg, dists))
  {
    g_pub_detected.publish(detected_msg);
    return;
  }

  auto detect_res = parkingDetect(angles_deg,
                                  dists,
                                  g_ransac_max_lines,
                                  g_ransac_max_iters,
                                  g_ransac_dist_thresh,
                                  g_ransac_min_inliers,
                                  g_parallel_angle_deg,
                                  g_orth_angle_deg);

  const bool is_u_shape = detect_res.first;
  const auto &lines = detect_res.second;

  if (!is_u_shape)
  {
    g_pub_detected.publish(detected_msg);
    return;
  }

  GoalResult goal = computeGoalFromLines(lines, g_min_width, g_min_depth, g_wall_offset);
  if (!goal.success)
  {
    g_pub_detected.publish(detected_msg);
    return;
  }

  detected_msg.data = true;
  g_pub_detected.publish(detected_msg);

  geometry_msgs::PoseStamped goal_msg;
  goal_msg.header.stamp = ros::Time::now();
  goal_msg.header.frame_id = g_goal_frame_id;
  goal_msg.pose.position.x = goal.x;
  goal_msg.pose.position.y = goal.y;
  goal_msg.pose.position.z = 0.0;

  // 각도는 사용하지 않음: orientation을 기본값(0,0,0,1)으로 설정
  goal_msg.pose.orientation.x = 0.0;
  goal_msg.pose.orientation.y = 0.0;
  goal_msg.pose.orientation.z = 0.0;
  goal_msg.pose.orientation.w = 1.0;

  g_pub_goal.publish(goal_msg);
  if (g_pub_goal_marker)
  {
    visualization_msgs::Marker marker;
    marker.header = goal_msg.header;
    marker.ns = "parking_goal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = goal_msg.pose;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.2f;
    marker.color.g = 0.8f;
    marker.color.b = 0.2f;
    marker.color.a = 0.9f;
    g_pub_goal_marker.publish(marker);
  }
  g_last_goal = goal_msg;

  ROS_INFO_THROTTLE(1.0,
                    "[parking_node] detected U-shape -> goal x=%.2f y=%.2f (frame=%s)",
                    goal.x, goal.y, g_goal_frame_id.c_str());
}

// -------------------- main --------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "parking_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<std::string>("scan_topic", g_scan_topic, std::string("/scan"));
  pnh.param<std::string>("detected_topic", g_detected_topic, std::string("/parking_detected"));
  pnh.param<std::string>("goal_topic", g_goal_topic, std::string("/parking_goal"));
  pnh.param<std::string>("goal_frame_id", g_goal_frame_id, std::string("base_link"));
  pnh.param<std::string>("goal_marker_topic", g_goal_marker_topic, std::string("/parking_goal_marker"));

  pnh.param<int>("ransac_max_lines", g_ransac_max_lines, 5);
  pnh.param<int>("ransac_max_iters", g_ransac_max_iters, 200);
  pnh.param<double>("ransac_dist_thresh", g_ransac_dist_thresh, 0.05);
  pnh.param<int>("ransac_min_inliers", g_ransac_min_inliers, 30);
  pnh.param<double>("parallel_angle_deg", g_parallel_angle_deg, 15.0);
  pnh.param<double>("orth_angle_deg", g_orth_angle_deg, 15.0);

  pnh.param<double>("min_width", g_min_width, 0.3);
  pnh.param<double>("min_depth", g_min_depth, 0.2);
  pnh.param<double>("wall_offset", g_wall_offset, 0.1);

  ROS_INFO("[parking_node] subscribe scan='%s'", ros::names::resolve(g_scan_topic).c_str());
  ROS_INFO("[parking_node] publish detected='%s', goal='%s' (frame=%s)",
           ros::names::resolve(g_detected_topic).c_str(),
           ros::names::resolve(g_goal_topic).c_str(),
           g_goal_frame_id.c_str());
  ROS_INFO("[parking_node] publish goal marker='%s'",
           ros::names::resolve(g_goal_marker_topic).c_str());

  g_sub_scan = nh.subscribe(g_scan_topic, 1, scanCallback);
  g_pub_detected = nh.advertise<std_msgs::Bool>(g_detected_topic, 1);
  g_pub_goal = nh.advertise<geometry_msgs::PoseStamped>(g_goal_topic, 1);
  g_pub_goal_marker = nh.advertise<visualization_msgs::Marker>(g_goal_marker_topic, 1);

  ros::Rate rate(15.0);
  while (ros::ok())
  {
    ros::spinOnce();
    process();
    rate.sleep();
  }

  return 0;
}
