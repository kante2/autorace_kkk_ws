#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <sstream>

namespace
{
// 토픽 이름
std::string g_detected_topic;
std::string g_target_topic;
std::string g_motor_topic;
std::string g_servo_topic;
std::string g_cluster_topic;

// 파라미터
double g_k_yaw = 0.9; // 이 값을 올린다.  -> 스티어 게인을 키울수 있다. 
double g_follow_speed_mps = 1.0;
double g_servo_center = 0.5;
double g_servo_min = 0.0;
double g_servo_max = 1.0;
double g_steer_sign = -1.0;
double g_motor_min_cmd = 0.0;
double g_motor_max_cmd = 2000.0;
double g_motor_gain = 900.0;
double g_marker_timeout_sec = 0.5;  // 타깃 타임아웃으로 재사용
double g_cluster_thresh_m = 0.50;   // 장애물 임계 거리 (m)
double g_cluster_timeout_sec = 1.0; // 클러스터 신선도 판단 시간
double g_avoid_hold_sec = 0.7;      // 최대 조향 유지 시간
double g_avoid_stop_sec = 1.0;      // 회피 진입 시 정지 시간 (고정)

// 상태
ros::Subscriber g_sub_detected;
ros::Subscriber g_sub_target;
ros::Subscriber g_sub_cluster;
ros::Publisher  g_pub_motor;
ros::Publisher  g_pub_servo;

bool g_detected = false;
bool g_have_target = false;
double g_target_x = 0.0;
double g_target_y = 0.0;
ros::Time g_last_target_time;

// 클러스터 상태
double g_latest_left_dist = -1.0;
double g_latest_right_dist = -1.0;
double g_latest_left_ang = -1.0;
double g_latest_right_ang = -1.0;
ros::Time g_last_cluster_time;

// 회피 상태
bool g_avoid_active = false;
bool g_avoid_left = false;
ros::Time g_avoid_start_time;

inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

void detectedCB(const std_msgs::Bool::ConstPtr& msg)
{
  g_detected = msg->data;
}

void targetCB(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  g_target_x = msg->point.x;
  g_target_y = msg->point.y;
  g_have_target = true;
  g_last_target_time = ros::Time::now();
}

void clusterInfoCB(const std_msgs::String::ConstPtr& msg)
{
  g_latest_left_dist = -1.0;
  g_latest_right_dist = -1.0;
  g_latest_left_ang = -1.0;
  g_latest_right_ang = -1.0;

  std::istringstream ss(msg->data);
  std::string token;
  while (std::getline(ss, token, ','))
  {
    auto pos = token.find(':');
    if (pos == std::string::npos) continue;
    std::string ang_str = token.substr(0, pos);
    std::string val = token.substr(pos + 1);
    try
    {
      double ang_deg = std::stod(ang_str);
      double dist = std::stod(val);

      // 각도 기준: 후방=0, 전방=180, 반시계+. 요구사항: 150~180 -> 오른쪽, 180~210 -> 왼쪽
      if (ang_deg >= 150.0 && ang_deg <= 180.0)
      {
        g_latest_right_dist = (g_latest_right_dist < 0.0) ? dist : std::min(g_latest_right_dist, dist);
        g_latest_right_ang  = ang_deg;
      }
      else if (ang_deg >= 180.0 && ang_deg <= 210.0)
      {
        g_latest_left_dist = (g_latest_left_dist < 0.0) ? dist : std::min(g_latest_left_dist, dist);
        g_latest_left_ang  = ang_deg;
      }
    }
    catch (...) {}
  }
  g_last_cluster_time = ros::Time::now();
}
}  // namespace

// main_node에서 호출
void mission_labacorn_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[mission_labacorn] init (consume marker array)");

  pnh.param<std::string>("labacorn_detected_topic", g_detected_topic, std::string("/labacorn_detected"));
  pnh.param<std::string>("labacorn_target_topic", g_target_topic, std::string("/labacorn/target"));
  pnh.param<std::string>("motor_topic", g_motor_topic, std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic", g_servo_topic, std::string("/commands/servo/position"));
  pnh.param<std::string>("cluster_topic", g_cluster_topic, std::string("/cluster_info"));

  pnh.param<double>("yaw_gain", g_k_yaw, 10); // 5 good  
  pnh.param<double>("follow_speed_mps", g_follow_speed_mps, 1.0);
  pnh.param<double>("servo_center", g_servo_center, 0.5);
  pnh.param<double>("servo_min", g_servo_min, 0.0);
  pnh.param<double>("servo_max", g_servo_max, 1.0);
  pnh.param<double>("steer_sign", g_steer_sign, 1.0);
  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);
  pnh.param<double>("motor_max_cmd", g_motor_max_cmd, 900.0);
  pnh.param<double>("motor_gain", g_motor_gain, 900.0);
  pnh.param<double>("marker_timeout_sec", g_marker_timeout_sec, 0.5);  // target timeout
  pnh.param<double>("cluster_thresh_m", g_cluster_thresh_m, 0.43);
  pnh.param<double>("cluster_timeout_sec", g_cluster_timeout_sec, 1.0);
  pnh.param<double>("avoid_hold_sec", g_avoid_hold_sec, 0.7);
  pnh.param<double>("avoid_stop_sec", g_avoid_stop_sec, 1.0);

  ROS_INFO("[mission_labacorn] subscribe detected='%s', target='%s'",
           ros::names::resolve(g_detected_topic).c_str(),
           ros::names::resolve(g_target_topic).c_str());
  ROS_INFO("[mission_labacorn] subscribe cluster='%s' (left/right dist)",
           ros::names::resolve(g_cluster_topic).c_str());
  ROS_INFO("[mission_labacorn] publish motor='%s', servo='%s'",
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str());

  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_pub_servo = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);
  g_sub_detected = nh.subscribe(g_detected_topic, 1, detectedCB);
  g_sub_target = nh.subscribe(g_target_topic, 1, targetCB);
  g_sub_cluster = nh.subscribe(g_cluster_topic, 1, clusterInfoCB);

  g_detected = false;
  g_have_target = false;
  g_last_target_time = ros::Time(0);
  g_latest_left_dist = g_latest_right_dist = -1.0;
  g_last_cluster_time = ros::Time(0);
  g_avoid_active = false;
}

void mission_labacorn_step()
{
  ros::Time now = ros::Time::now();
  bool fresh_target = false;
  if (g_have_target && !g_last_target_time.isZero())
  {
    fresh_target = (now - g_last_target_time).toSec() <= g_marker_timeout_sec;
  }

  double motor_out = 0.0;
  double servo_out = g_servo_center;

  // 회피 트리거: 최근 클러스터가 임계치보다 가까우면 회피 모드 진입
  if (!g_avoid_active)
  {
    bool cluster_fresh = (!g_last_cluster_time.isZero()) &&
                         ((now - g_last_cluster_time).toSec() <= g_cluster_timeout_sec);
    bool left_close  = cluster_fresh && g_latest_left_dist  > 0.0 && g_latest_left_dist  < g_cluster_thresh_m;
    bool right_close = cluster_fresh && g_latest_right_dist > 0.0 && g_latest_right_dist < g_cluster_thresh_m;

    if (left_close || right_close)
    {
      g_avoid_active = true;
      g_avoid_left = left_close; // left가 우선
      g_avoid_start_time = now;
      ROS_WARN("[mission_labacorn] cluster too close (L=%.3f, R=%.3f) -> avoid %s with max steer",
               g_latest_left_dist, g_latest_right_dist,
               g_avoid_left ? "LEFT" : "RIGHT");
    }
  }

  // 회피 실행
  if (g_avoid_active)
  {
    double elapsed = (now - g_avoid_start_time).toSec();
    // 회피: 최대 조향 유지하며 1) 후진(hold_sec) -> 2) 정지(stop_sec)
    if (elapsed < g_avoid_hold_sec)
    {
      motor_out = -900.0;  // 고정 후진
    }
    else if (elapsed < g_avoid_hold_sec + g_avoid_stop_sec)
    {
      motor_out = 0.0;  // 정지
    }
    else
    {
      g_avoid_active = false;
      g_latest_left_dist = g_latest_right_dist = -1.0; // 소비 완료
    }
    // 최대 조향: 왼쪽 장애물->왼쪽(0), 오른쪽 장애물->오른쪽(1)로 강제
    servo_out = g_avoid_left ? 0.0 : 1.0;

    std_msgs::Float64 motor_msg; motor_msg.data = motor_out;
    std_msgs::Float64 servo_msg; servo_msg.data = servo_out;
    g_pub_motor.publish(motor_msg);
    g_pub_servo.publish(servo_msg);

    ROS_INFO_THROTTLE(0.2,
                      "[mission_labacorn][avoid] dir=%s elapsed=%.2f motor=%.1f servo=%.3f",
                      g_avoid_left ? "LEFT" : "RIGHT", elapsed, motor_out, servo_out);
    return;
  }

  // 기본: 항상 900으로 주행. 타깃이 신선하면 조향, 아니면 중앙 유지.
  double yaw = 0.0;
  double steer_cmd = 0.0;
  if (g_detected && fresh_target)
  {
    yaw = std::atan2(g_target_y, 1.0);
    steer_cmd = std::tanh(g_k_yaw * yaw);
  }

  double steer_norm = clamp(steer_cmd, -1.0, 1.0);
  steer_norm *= (-g_steer_sign);
  double servo_range = std::min(g_servo_center - g_servo_min,
                                g_servo_max - g_servo_center);
  servo_out = clamp(g_servo_center + steer_norm * servo_range,
                    g_servo_min, g_servo_max);

  // 라바콘 주행 속도: 고정 900 (가감속 없이 상수 속도)
  motor_out = 900.0;

  std_msgs::Float64 motor_msg; motor_msg.data = motor_out;
  std_msgs::Float64 servo_msg; servo_msg.data = servo_out;
  g_pub_motor.publish(motor_msg);
  g_pub_servo.publish(servo_msg);

  ROS_INFO_THROTTLE(0.5,
                    "[mission_labacorn] detected=%d fresh_target=%d yaw=%.3f steer=%.3f motor=%.1f servo=%.3f (target=%.2f,%.2f)",
                    static_cast<int>(g_detected),
                    static_cast<int>(fresh_target),
                    yaw,
                    steer_cmd,
                    motor_out,
                    servo_out,
                    g_target_x, g_target_y);
}
