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
std::string g_goal_frame_id;
std::string g_goal_marker_topic;
std::string g_lines_marker_topic;
std::string g_enable_topic;

// 검출 파라미터 (튜닝 가능)
int    g_ransac_max_lines   = 5;     // RANSAC 시도할 최대 선 개수 (크면 탐색 ↑, 시간 ↑)
int    g_ransac_max_iters   = 200;   // RANSAC 반복 횟수 (크면 안정 ↑, 시간 ↑)
double g_ransac_dist_thresh = 0.035; // 점-선 거리 임계 (m), 더 느슨하게 잡아 포인트 부족 방지
int    g_ransac_min_inliers = 25;    // 선 인정 최소 인라이어 수, 포인트 적을 때도 통과하도록 낮춤
int    g_max_line_inliers   = 100;   // 한 선이 먹을 수 있는 최대 포인트 수 (교선으로 길게 잡히는 것 제한)
double g_parallel_angle_deg = 7.0;   // 평행 판정 허용 각도 (deg), 실제 각도 편차 반영 위해 완화
double g_orth_angle_deg     = 7.0;   // 직교 판정 허용 각도 (deg), 실제 각도 편차 반영 위해 완화
int    g_strict_min_inliers = 50;    // U-shape 최종 인정 인라이어 하한, 미검 방지 위해 낮춤

// 슬롯 크기 파라미터 (튜닝 가능)
double g_min_width   = 0.40; // 최소 주차폭 (m) - 40cm 이상
double g_max_width   = 1.60; // 최대 주차폭 (m) - 넉넉히 열어 실제 폭 1m 내외 허용
double g_min_depth   = 0.30; // 최소 주차깊이 (m) - 30cm 이상
double g_max_depth   = 1.50; // 최대 주차깊이 (m) - 넉넉히 열어 실제 깊이 허용
double g_wall_offset = 0.0;  // 벽 기준 목표점 y 오프셋 (m), 필요 시 ±0.05 내 조정

// 상태
ros::Subscriber g_sub_scan;
ros::Publisher  g_pub_detected;
ros::Publisher  g_pub_goal_marker;
ros::Publisher  g_pub_lines_marker;
sensor_msgs::LaserScan::ConstPtr g_last_scan;
bool g_enabled = true;

void enableCB(const std_msgs::Bool::ConstPtr& msg)
{
  g_enabled = msg->data;
}

// -------------------- 헬퍼 --------------------
void publishDeleteMarkers()
{
  const ros::Time now = ros::Time::now();
  if (g_pub_goal_marker)
  {
    visualization_msgs::Marker m;
    m.header.stamp = now;
    m.header.frame_id = g_goal_frame_id;
    m.ns = "parking_goal";
    m.id = 0;
    m.action = visualization_msgs::Marker::DELETE;
    g_pub_goal_marker.publish(m);
  }
  if (g_pub_lines_marker)
  {
    visualization_msgs::Marker m;
    m.header.stamp = now;
    m.header.frame_id = g_goal_frame_id;
    m.ns = "parking_lines";
    m.id = 0;
    m.action = visualization_msgs::Marker::DELETE;
    g_pub_lines_marker.publish(m);
  }
}

// -------------------- 콜백 --------------------
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  if (!g_enabled) return;
  g_last_scan = msg;
}

// -------------------- 메인 루프 처리 --------------------
void process()
{
  if (!g_enabled) {
    publishDeleteMarkers();
    return;
  }

  std_msgs::Bool detected_msg;
  detected_msg.data = false;

  if (!g_last_scan)
  {
    g_pub_detected.publish(detected_msg);
    publishDeleteMarkers();
    return;
  }

  std::vector<double> angles_deg;
  std::vector<double> dists;
  if (!preprocessLidar(*g_last_scan, angles_deg, dists))
  {
    g_pub_detected.publish(detected_msg);
    publishDeleteMarkers();
    return;
  }

  auto detect_res = parkingDetect(angles_deg,
                                  dists,
                                  g_ransac_max_lines,
                                  g_ransac_max_iters,
                                  g_ransac_dist_thresh,
                                  g_ransac_min_inliers,
                                  g_max_line_inliers,
                                  g_parallel_angle_deg,
                                  g_orth_angle_deg);

  const bool is_u_shape = detect_res.first;
  const auto &lines = detect_res.second;

  if (!is_u_shape)
  {
    g_pub_detected.publish(detected_msg);
    publishDeleteMarkers();
    return;
  }

  // 선 3개 모두 인라이어 수가 충분히 많아야 인정 (부분 가려짐 방지)
  int min_inliers_found = std::min({lines[0].num_inliers,
                                    lines[1].num_inliers,
                                    lines[2].num_inliers});
  if (min_inliers_found < g_strict_min_inliers)
  {
    g_pub_detected.publish(detected_msg);
    publishDeleteMarkers();
    return;
  }

  GoalResult goal = computeGoalFromLines(lines,
                                         g_min_width,
                                         g_min_depth,
                                         g_wall_offset,
                                         g_max_width,
                                         g_max_depth);
  if (!goal.success)
  {
    g_pub_detected.publish(detected_msg);
    publishDeleteMarkers();
    return;
  }

  detected_msg.data = true;
  g_pub_detected.publish(detected_msg);

  // RViz 시각화는 U자 검출 성공 + goal 계산 성공 시에만 퍼블리시
  if (g_pub_goal_marker)
  {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = g_goal_frame_id;
    marker.ns = "parking_goal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = goal.x;
    marker.pose.position.y = goal.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
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
    lines_marker.header.stamp = ros::Time::now();
    lines_marker.header.frame_id = g_goal_frame_id;
    lines_marker.ns = "parking_lines";
    lines_marker.id = 0;
    lines_marker.type = visualization_msgs::Marker::LINE_LIST;
    lines_marker.action = visualization_msgs::Marker::ADD;
    lines_marker.scale.x = 0.02;  // 선 두께
    lines_marker.color.r = 0.0f;
    lines_marker.color.g = 0.4f;
    lines_marker.color.b = 1.0f;
    lines_marker.color.a = 0.8f;

    const double half_len = 1.0;  // 각 선을 +/-1m로 표시해 시각화 범위를 줄임
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
  pnh.param<std::string>("goal_frame_id", g_goal_frame_id, std::string("base_link"));
  pnh.param<std::string>("goal_marker_topic", g_goal_marker_topic, std::string("/parking_goal_marker"));
  pnh.param<std::string>("lines_marker_topic", g_lines_marker_topic, std::string("/parking_lines"));
  pnh.param<std::string>("enable_topic", g_enable_topic, std::string("/perception/parking/enable"));

  // --- 튜닝 파라미터 ---
  pnh.param<int>("ransac_max_lines", g_ransac_max_lines, 5);             // 선 탐색 수: ↑ 시 탐색↑/시간↑
  pnh.param<int>("ransac_max_iters", g_ransac_max_iters, 200);           // 반복 횟수: ↑ 시 안정↑/시간↑
  pnh.param<double>("ransac_dist_thresh", g_ransac_dist_thresh, 0.035);  // 점-선 거리 임계: 느슨하게
  pnh.param<int>("ransac_min_inliers", g_ransac_min_inliers, 25);        // 선 인정 최소 점수: 낮춰서 미검 방지
  pnh.param<int>("max_line_inliers", g_max_line_inliers, 100);           // 한 선이 먹을 수 있는 최대 포인트 수
  pnh.param<double>("parallel_angle_deg", g_parallel_angle_deg, 7.0);    // 평행 허용 각도: 완화
  pnh.param<double>("orth_angle_deg", g_orth_angle_deg, 7.0);            // 직교 허용 각도: 완화
  pnh.param<int>("strict_min_inliers", g_strict_min_inliers, 50);        // U-shape 최종 인라이어 하한: 낮춤

  pnh.param<double>("min_width", g_min_width, 0.40);   // 최소 주차폭: 0.4m 이상
  pnh.param<double>("max_width", g_max_width, 1.60);   // 최대 주차폭: 1.6m 이하(실제 폭 여유)
  pnh.param<double>("min_depth", g_min_depth, 0.30);   // 최소 주차깊이: 0.3m 이상
  pnh.param<double>("max_depth", g_max_depth, 1.50);   // 최대 주차깊이: 1.5m 이하(실제 깊이 여유)
  pnh.param<double>("wall_offset", g_wall_offset, 0.00); // 목표점 y 오프셋: 필요 시 ±0.05 조정

  ROS_INFO("[parking_node] subscribe scan='%s'", ros::names::resolve(g_scan_topic).c_str());
  ROS_INFO("[parking_node] publish detected='%s'",
           ros::names::resolve(g_detected_topic).c_str());
  ROS_INFO("[parking_node] publish goal marker='%s'",
           ros::names::resolve(g_goal_marker_topic).c_str());
  ROS_INFO("[parking_node] publish lines marker='%s'",
           ros::names::resolve(g_lines_marker_topic).c_str());

  ros::Subscriber enable_sub = nh.subscribe(g_enable_topic, 1, enableCB);
  g_sub_scan = nh.subscribe(g_scan_topic, 1, scanCallback);
  g_pub_detected = nh.advertise<std_msgs::Bool>(g_detected_topic, 1);
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
