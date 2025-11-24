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
std::string g_lines_marker_topic;

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
ros::Publisher  g_pub_lines_marker;
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

  // 라인 시각화 (검출된 선분들)
  if (g_pub_lines_marker)
  {
    visualization_msgs::Marker lines_marker;
    lines_marker.header = goal_msg.header;
    lines_marker.ns = "parking_lines";
    lines_marker.id = 0;
    lines_marker.type = visualization_msgs::Marker::LINE_LIST;
    lines_marker.action = visualization_msgs::Marker::ADD;
    lines_marker.scale.x = 0.02;  // 선 두께
    lines_marker.color.r = 0.0f;
    lines_marker.color.g = 0.4f;
    lines_marker.color.b = 1.0f;
    lines_marker.color.a = 0.8f;

    const double half_len = 2.0;  // 각 선을 +/-2m로 표시
    for (std::size_t i = 0; i < lines.size() && i < 3; ++i)
    {
      const auto& ln = lines[i];
      // 선의 방향벡터(t)는 노멀(a,b)에 직교: t = (-b, a)
      const double tx = -ln.b;
      const double ty = ln.a;
      const double norm = std::hypot(tx, ty);
      if (norm < 1e-6) continue;
      const double ux = tx / norm;
      const double uy = ty / norm;

      geometry_msgs::Point p1, p2;
      p1.x = ln.centroid_x + ux * half_len;
      p1.y = ln.centroid_y + uy * half_len;
      p1.z = 0.0;
      p2.x = ln.centroid_x - ux * half_len;
      p2.y = ln.centroid_y - uy * half_len;
      p2.z = 0.0;
      lines_marker.points.push_back(p1);
      lines_marker.points.push_back(p2);
    }

    g_pub_lines_marker.publish(lines_marker);
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
  pnh.param<std::string>("lines_marker_topic", g_lines_marker_topic, std::string("/parking_lines"));

  pnh.param<int>("ransac_max_lines", g_ransac_max_lines, 4);
  pnh.param<int>("ransac_max_iters", g_ransac_max_iters, 100);
  pnh.param<double>("ransac_dist_thresh", g_ransac_dist_thresh, 0.03);
  pnh.param<int>("ransac_min_inliers", g_ransac_min_inliers, 30);
  pnh.param<double>("parallel_angle_deg", g_parallel_angle_deg, 10.0);
  pnh.param<double>("orth_angle_deg", g_orth_angle_deg, 10.0);

  pnh.param<double>("min_width", g_min_width, 0.4);
  pnh.param<double>("min_depth", g_min_depth, 0.3);
  pnh.param<double>("wall_offset", g_wall_offset, 0.05);

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
  g_pub_lines_marker = nh.advertise<visualization_msgs::Marker>(g_lines_marker_topic, 1);

  ros::Rate rate(15.0);
  while (ros::ok())
  {
    ros::spinOnce();
    process();
    rate.sleep();
  }

  return 0;
}
