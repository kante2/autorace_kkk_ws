#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <sstream>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "clustering_visualization.hpp"
#include "lidar_preprocessing_rotary.hpp"
#include "obstacle_detect.hpp"

// -------------------- 전역 --------------------
std::string g_scan_topic;
std::string g_detected_topic;
std::string g_marker_topic;
std::string g_centroids_topic;
std::string g_enable_topic;

double g_eps = 0.3;
int g_min_pts = 7;
int g_max_pts = -1;
double g_close_dist = 1.0;  // 이 거리 이내 클러스터 존재 시 감지 true
double g_dynamic_speed_thresh = 0.2;  // 상대 속도 문턱 (m/s)
double g_match_max_angle_deg = 15.0;  // 이전-현재 클러스터 매칭 허용 각도
double g_match_max_dist = 1.0;        // 이전-현재 클러스터 매칭 허용 거리 차이 (m)

ros::Subscriber g_sub_scan;
ros::Publisher g_pub_detected;
ros::Publisher g_pub_centroids;
ros::Subscriber g_sub_motor;
ros::Subscriber g_sub_servo;
std::unique_ptr<ClusterVisualizer> g_visualizer;

double g_motor_cmd = 0.0;
double g_servo_pos = 0.5;
bool g_enabled = true;

void enableCB(const std_msgs::Bool::ConstPtr& msg)
{
  g_enabled = msg->data;
}

std::vector<ClusterCentroid> g_prev_centroids;
ros::Time g_prev_stamp;

// -------------------- 유틸 --------------------
inline double normalizeAngleDeg(double ang)
{
  double res = std::fmod(ang, 360.0);
  if (res < 0.0)
  {
    res += 360.0;
  }
  return res;
}

inline double shortestAngleDiffDeg(double a, double b)
{
  double diff = normalizeAngleDeg(a) - normalizeAngleDeg(b);
  if (diff > 180.0)
  {
    diff -= 360.0;
  }
  else if (diff < -180.0)
  {
    diff += 360.0;
  }
  return diff;
}

double servoToHeadingDeg(double servo_pos)
{
  // 0 -> -20 deg, 0.5 -> 0 deg, 1 -> +20 deg, 기본 진행 방향 180 deg (전방)
  const double steering_deg = (servo_pos - 0.5) * 40.0;
  return normalizeAngleDeg(180.0 + steering_deg);
}

double motorCmdToSpeedMps(double motor_cmd)
{
  // 1000 명령당 0.26 m/s
  return motor_cmd * 0.00026;
}
std::vector<bool> classifyDynamics(const std::vector<ClusterCentroid> &curr,
                                   const ros::Time &stamp)
{
  std::vector<bool> dynamic(curr.size(), false);

  // 초기화/예외 처리 ---------------------------------
  if (curr.empty())
  {
    g_prev_centroids.clear();
    g_prev_stamp = stamp;
    return dynamic;
  }

  if (g_prev_centroids.empty() || g_prev_stamp.isZero())
  {
    g_prev_centroids = curr;
    g_prev_stamp = stamp;
    return dynamic;
  }

  const double dt = (stamp - g_prev_stamp).toSec();
  if (dt <= 0.0)
  {
    g_prev_centroids = curr;
    g_prev_stamp = stamp;
    return dynamic;
  }

  // ego 상태 ----------------------------------------
  const double ego_speed   = motorCmdToSpeedMps(g_motor_cmd);        // m/s
  const double heading_deg = servoToHeadingDeg(g_servo_pos);
  const double heading_rad = heading_deg * M_PI / 180.0;

  const double vx_ego = ego_speed * std::cos(heading_rad);
  const double vy_ego = ego_speed * std::sin(heading_rad);

  // 각 클러스터에 대해 --------------------------------
  for (std::size_t i = 0; i < curr.size(); ++i)
  {
    const double ang_curr_deg = normalizeAngleDeg(curr[i].angle + 360.0);
    const double ang_curr_rad = ang_curr_deg * M_PI / 180.0;
    const double dist_curr    = curr[i].distance;

    // --- 이전 프레임 클러스터 중 best match 찾기 (각도+거리) ---
    int    best_idx   = -1;
    double best_score = std::numeric_limits<double>::max();

    for (std::size_t j = 0; j < g_prev_centroids.size(); ++j)
    {
      const double ang_prev_deg = normalizeAngleDeg(g_prev_centroids[j].angle + 360.0);
      const double dist_prev    = g_prev_centroids[j].distance;

      const double d_ang  = std::fabs(shortestAngleDiffDeg(ang_curr_deg, ang_prev_deg));
      const double d_dist = std::fabs(dist_curr - dist_prev);

      if (d_ang > g_match_max_angle_deg || d_dist > g_match_max_dist)
        continue;

      const double score = d_ang + d_dist;
      if (score < best_score)
      {
        best_score = score;
        best_idx   = static_cast<int>(j);
      }
    }

    if (best_idx < 0)
      continue;  // 매칭 실패 → 일단 정적으로 둠

    // --- 이전 프레임 좌표 (polar -> cartesian) ---
    const double ang_prev_deg = normalizeAngleDeg(g_prev_centroids[best_idx].angle + 360.0);
    const double ang_prev_rad = ang_prev_deg * M_PI / 180.0;
    const double dist_prev    = g_prev_centroids[best_idx].distance;

    const double x_prev = dist_prev * std::cos(ang_prev_rad);
    const double y_prev = dist_prev * std::sin(ang_prev_rad);
    const double x_curr = dist_curr * std::cos(ang_curr_rad);
    const double y_curr = dist_curr * std::sin(ang_curr_rad);

    // --- 관측된 2D 속도 (라이다 좌표계) ---
    const double vx_obs = (x_curr - x_prev) / dt;
    const double vy_obs = (y_curr - y_prev) / dt;

    // --- ego 속도 보정 → 상대 속도 ---
    const double vx_rel = vx_obs - vx_ego;
    const double vy_rel = vy_obs - vy_ego;

    const double speed_rel = std::sqrt(vx_rel * vx_rel + vy_rel * vy_rel);  // m/s

    if (speed_rel > g_dynamic_speed_thresh)
      dynamic[i] = true;

    // 튜닝용 디버그 (원하면 잠깐 켜서 값 확인)
    /*
    ROS_INFO_THROTTLE(0.5,
      "[dyn] i=%zu, dt=%.3f, v_obs=(%.2f, %.2f), v_ego=(%.2f, %.2f), "
      "v_rel=(%.2f, %.2f), |v_rel|=%.2f, dyn=%d",
      i, dt, vx_obs, vy_obs, vx_ego, vy_ego,
      vx_rel, vy_rel, speed_rel, (int)dynamic[i]);
    */
  }

  g_prev_centroids = curr;
  g_prev_stamp = stamp;
  return dynamic;
}


void motorCallback(const std_msgs::Float64::ConstPtr &msg)
{
  g_motor_cmd = msg->data;
}

void servoCallback(const std_msgs::Float64::ConstPtr &msg)
{
  g_servo_pos = msg->data;
}

// -------------------- 콜백 --------------------
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  if (!g_enabled) return;

  const LidarProcessingResult pre = preprocessLidar(*scan_msg);
  DetectionResult det =
      detect(pre.angle_ranges_deg, pre.dist_ranges, g_eps, g_min_pts, g_max_pts);

  // 동적/정적 분류 (이전 프레임 대비 상대 속도)
  const std::vector<bool> dynamic_flags = classifyDynamics(det.centroids, scan_msg->header.stamp);

  std::vector<ClusterCentroid> close_centroids;
  std::vector<bool> close_dynamic_flags;
  if (det.detected)
  {
    for (std::size_t i = 0; i < det.centroids.size(); ++i)
    {
      const auto &c = det.centroids[i];
      if (c.distance <= g_close_dist)
      {
        close_centroids.push_back(c);
        close_dynamic_flags.push_back(dynamic_flags[i]);
      }
    }
  }
  bool has_close_dynamic = false;
  for (bool dyn : close_dynamic_flags)
  {
    if (dyn)
    {
      has_close_dynamic = true;
      break;
    }
  }

  std_msgs::Bool msg;
  msg.data = has_close_dynamic;
  g_pub_detected.publish(msg);

  // g_close_dist 이내 + 동적 센트로이드만 전송
  if (has_close_dynamic)
  {
    std_msgs::Float32MultiArray centroids_msg;
    centroids_msg.data.reserve(close_centroids.size() * 2);
    for (std::size_t i = 0; i < close_centroids.size(); ++i)
    {
      if (!close_dynamic_flags[i])
      {
        continue;
      }
      const auto &c = close_centroids[i];
      const double adjusted_angle = c.angle + 360.0;  // 라이다가 뒤집힌 경우 단순 오프셋
      centroids_msg.data.push_back(static_cast<float>(adjusted_angle));
      centroids_msg.data.push_back(static_cast<float>(c.distance));
    }
    if (!centroids_msg.data.empty())
    {
      g_pub_centroids.publish(centroids_msg);
    }
  }

  if (g_visualizer && det.detected)
  {
    g_visualizer->publish(det, dynamic_flags);
  }

  std::size_t dyn_count = 0U;
  for (bool f : dynamic_flags)
  {
    dyn_count += static_cast<std::size_t>(f);
  }

  if (!dynamic_flags.empty())
  {
    std::ostringstream oss;
    oss << "[rotary_node] dynamic_count=" << dyn_count;
    for (std::size_t i = 0, idx = 0; i < det.centroids.size(); ++i)
    {
      if (!dynamic_flags[i])
      {
        continue;
      }
      const double adjusted_angle = det.centroids[i].angle + 360.0;
      oss << " #" << idx++ << "=( " << std::fixed << std::setprecision(1) << adjusted_angle
          << " deg, " << std::setprecision(2) << det.centroids[i].distance << " m )";
    }
    ROS_INFO_THROTTLE(1.0, "%s", oss.str().c_str());
  }
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
  pnh.param<std::string>("centroids_topic", g_centroids_topic, std::string("rotary/centroids"));
  pnh.param<std::string>("enable_topic", g_enable_topic, std::string("/perception/rotary/enable"));

  pnh.param<double>("eps", g_eps, 0.3);
  pnh.param<int>("min_pts", g_min_pts,4);
  pnh.param<int>("max_pts", g_max_pts, 20); // -1: no limit
  pnh.param<double>("close_dist", g_close_dist, 1.0);
  pnh.param<double>("dynamic_speed_thresh", g_dynamic_speed_thresh, 0.3);
  pnh.param<double>("match_max_angle_deg", g_match_max_angle_deg, 10.0);
  pnh.param<double>("match_max_dist", g_match_max_dist, 0.8);

  ROS_INFO("[rotary_node] subscribe scan='%s'", ros::names::resolve(g_scan_topic).c_str());
  ROS_INFO("[rotary_node] subscribe motor='%s', servo='%s'", "/commands/motor/speed",
           "/commands/servo/position");
  ROS_INFO("[rotary_node] publish detected='%s', marker='%s', centroids='%s'",
           ros::names::resolve(g_detected_topic).c_str(),
           ros::names::resolve(g_marker_topic).c_str(),
           ros::names::resolve(g_centroids_topic).c_str());

  g_pub_detected = nh.advertise<std_msgs::Bool>(g_detected_topic, 1);
  g_pub_centroids = nh.advertise<std_msgs::Float32MultiArray>(g_centroids_topic, 1);
  g_visualizer = std::make_unique<ClusterVisualizer>(g_marker_topic);

  ros::Subscriber enable_sub = nh.subscribe(g_enable_topic, 1, enableCB);
  g_sub_scan = nh.subscribe(g_scan_topic, 1, scanCallback);
  g_sub_motor = nh.subscribe("/commands/motor/speed", 1, motorCallback);
  g_sub_servo = nh.subscribe("/commands/servo/position", 1, servoCallback);

  ros::spin();
  return 0;
}
