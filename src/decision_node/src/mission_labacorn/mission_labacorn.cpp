#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace
{
// 토픽 이름
std::string g_detected_topic;
std::string g_target_topic;
std::string g_motor_topic;
std::string g_servo_topic;

// 파라미터
double g_k_yaw = 0.9; // 이 값을 올린다.  -> 스티어 게인을 키울수 있다. 
double g_follow_speed_mps = 1.0;
double g_servo_center = 0.5;
double g_servo_min = 0.0;
double g_servo_max = 1.0;
double g_steer_sign = -1.0;
double g_motor_min_cmd = 0.0;
double g_motor_max_cmd = 2000.0;
double g_motor_gain = 1500.0;
double g_marker_timeout_sec = 0.5;  // 타깃 타임아웃으로 재사용

// 상태
ros::Subscriber g_sub_detected;
ros::Subscriber g_sub_target;
ros::Publisher  g_pub_motor;
ros::Publisher  g_pub_servo;

bool g_detected = false;
bool g_have_target = false;
double g_target_x = 0.0;
double g_target_y = 0.0;
ros::Time g_last_target_time;

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
}  // namespace

// main_node에서 호출
void mission_labacorn_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[mission_labacorn] init (consume marker array)");

  pnh.param<std::string>("labacorn_detected_topic", g_detected_topic, std::string("/labacorn_detected"));
  pnh.param<std::string>("labacorn_target_topic", g_target_topic, std::string("/labacorn/target"));
  pnh.param<std::string>("motor_topic", g_motor_topic, std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic", g_servo_topic, std::string("/commands/servo/position"));

  pnh.param<double>("yaw_gain", g_k_yaw, 10); // 5 good  
  pnh.param<double>("follow_speed_mps", g_follow_speed_mps, 1.0);
  pnh.param<double>("servo_center", g_servo_center, 0.5);
  pnh.param<double>("servo_min", g_servo_min, 0.0);
  pnh.param<double>("servo_max", g_servo_max, 1.0);
  pnh.param<double>("steer_sign", g_steer_sign, 1.0);
  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);
  pnh.param<double>("motor_max_cmd", g_motor_max_cmd, 2000.0);
  pnh.param<double>("motor_gain", g_motor_gain, 1500.0);
  pnh.param<double>("marker_timeout_sec", g_marker_timeout_sec, 0.5);  // target timeout

  ROS_INFO("[mission_labacorn] subscribe detected='%s', target='%s'",
           ros::names::resolve(g_detected_topic).c_str(),
           ros::names::resolve(g_target_topic).c_str());
  ROS_INFO("[mission_labacorn] publish motor='%s', servo='%s'",
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str());

  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_pub_servo = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);
  g_sub_detected = nh.subscribe(g_detected_topic, 1, detectedCB);
  g_sub_target = nh.subscribe(g_target_topic, 1, targetCB);

  g_detected = false;
  g_have_target = false;
  g_last_target_time = ros::Time(0);
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

  // 감지가 없거나 타임아웃이면 lane 모드 등에 맡기고 아무 것도 퍼블리시하지 않음
  if (!(g_detected && fresh_target))
  {
    ROS_INFO_THROTTLE(0.5,
                      "[mission_labacorn] detected=%d fresh_target=%d -> skip publish",
                      static_cast<int>(g_detected),
                      static_cast<int>(fresh_target));
    return;
  }

  // x는 무시하고 횡방향(y) 오프셋만으로 헤딩을 잡는다.
  // lookahead을 1.0m로 가정해 atan2(y, 1.0) 형태로 y만 반영.
  double yaw = std::atan2(g_target_y, 1.0);
  double steer_cmd = std::tanh(g_k_yaw * yaw);
  double speed_cmd = g_follow_speed_mps;

  double steer_norm = clamp(steer_cmd, -1.0, 1.0);
  steer_norm *= (-g_steer_sign);
  double servo_range = std::min(g_servo_center - g_servo_min,
                                g_servo_max - g_servo_center);
  servo_out = clamp(g_servo_center + steer_norm * servo_range,
                    g_servo_min, g_servo_max);

  motor_out = clamp(g_motor_gain * speed_cmd, g_motor_min_cmd, g_motor_max_cmd);

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
