// mission_rotary_toMain.cpp
// main_node에서 호출: /rotary_detected(Bool) 상태에 따라 속도 제어, 서보는 직진 유지

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <string>
#include <cmath>

// -------------------- 유틸 --------------------
inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- 전역 상태 --------------------
// 토픽 이름
static std::string g_topic_rotary_detected;
static std::string g_motor_topic;
static std::string g_servo_topic;

// 서보 파라미터 (직진 유지)
static double g_servo_center = 0.5;
static double g_servo_min    = 0.0;
static double g_servo_max    = 1.0;
static double g_steer_sign   = -1.0;

// 속도/모터 파라미터
static double g_cruise_speed_mps = 1.0;   // 감지 없을 때
static double g_slow_speed_mps   = 0.5;   // 감지 있을 때
static double g_speed_step_mps   = 0.1;   // 한 스텝당 변화량
static double g_motor_min_cmd    = 0.0;
static double g_motor_max_cmd    = 2000.0;
static double g_motor_gain       = 1500.0; // m/s -> 모터 명령

// 감지 타임아웃
static double   g_detect_timeout_sec = 0.5;
static bool     g_rotary_detected    = false;
static ros::Time g_last_detect_time;

// 퍼블리셔
static ros::Publisher g_pub_motor;
static ros::Publisher g_pub_servo;
static ros::Subscriber g_sub_rotary;

// 내부 상태
static double g_current_speed_mps = 0.0;

// -------------------- 콜백 --------------------
static void rotaryDetectedCB(const std_msgs::Bool::ConstPtr& msg)
{
  g_rotary_detected = msg->data;
  g_last_detect_time = ros::Time::now();
}

// -------------------- init --------------------
void mission_rotary_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[mission_rotary] mission_rotary_init()");

  // 토픽
  pnh.param<std::string>("rotary_detected_topic",
                         g_topic_rotary_detected,
                         std::string("/rotary_detected"));
  pnh.param<std::string>("motor_topic",
                         g_motor_topic,
                         std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic",
                         g_servo_topic,
                         std::string("/commands/servo/position"));

  // 서보 파라미터
  pnh.param<double>("servo_center", g_servo_center, 0.5);
  pnh.param<double>("servo_min",    g_servo_min,    0.0);
  pnh.param<double>("servo_max",    g_servo_max,    1.0);
  pnh.param<double>("steer_sign",   g_steer_sign,  -1.0);

  // 속도/모터 파라미터
  pnh.param<double>("cruise_speed_mps", g_cruise_speed_mps, 1.0);
  pnh.param<double>("slow_speed_mps",   g_slow_speed_mps,   0.5);
  pnh.param<double>("speed_step_mps",   g_speed_step_mps,   0.1);
  pnh.param<double>("motor_min_cmd",    g_motor_min_cmd,    0.0);
  pnh.param<double>("motor_max_cmd",    g_motor_max_cmd,    2000.0);
  pnh.param<double>("motor_gain",       g_motor_gain,       1500.0);

  // 타임아웃
  pnh.param<double>("detect_timeout_sec", g_detect_timeout_sec, 0.5);

  ROS_INFO("[mission_rotary] subscribe rotary_detected='%s'",
           ros::names::resolve(g_topic_rotary_detected).c_str());
  ROS_INFO("[mission_rotary] publish motor='%s', servo='%s'",
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str());

  g_sub_rotary = nh.subscribe(g_topic_rotary_detected, 1, rotaryDetectedCB);
  g_pub_motor  = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_pub_servo  = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);

  g_current_speed_mps = g_cruise_speed_mps;
  g_last_detect_time = ros::Time(0);
  g_rotary_detected = false;

  ROS_INFO("[mission_rotary] mission_rotary_init done");
}

// -------------------- step --------------------
void mission_rotary_step()
{
  ros::Time now = ros::Time::now();

  // 타임아웃 검사: 오래 감지 업데이트 없으면 감지 false
  bool detected = g_rotary_detected;
  if (!g_last_detect_time.isZero()) {
    double dt = (now - g_last_detect_time).toSec();
    if (dt > g_detect_timeout_sec) {
      detected = false;
    }
  } else {
    detected = false;
  }

  // 목표 속도 결정
  double target_speed = detected ? g_slow_speed_mps : g_cruise_speed_mps;

  // 속도 램프
  if (g_current_speed_mps > target_speed) {
    g_current_speed_mps = std::max(target_speed, g_current_speed_mps - g_speed_step_mps);
  } else if (g_current_speed_mps < target_speed) {
    g_current_speed_mps = std::min(target_speed, g_current_speed_mps + g_speed_step_mps);
  }

  // 서보: 직진
  double steer_cmd = 0.0;
  double steer_norm = clamp(steer_cmd, -1.0, 1.0);
  steer_norm *= (-g_steer_sign);

  double servo_range = std::min(g_servo_center - g_servo_min,
                                g_servo_max - g_servo_center);
  double servo_hw = g_servo_center + steer_norm * servo_range;
  servo_hw = clamp(servo_hw, g_servo_min, g_servo_max);

  // 모터: m/s -> 명령
  double motor_cmd = g_motor_gain * g_current_speed_mps;
  motor_cmd = clamp(motor_cmd, g_motor_min_cmd, g_motor_max_cmd);

  // 퍼블리시
  std_msgs::Float64 motor_msg;
  std_msgs::Float64 servo_msg;
  motor_msg.data = motor_cmd;
  servo_msg.data = servo_hw;

  g_pub_motor.publish(motor_msg);
  g_pub_servo.publish(servo_msg);

  ROS_INFO_THROTTLE(0.5,
    "[mission_rotary] detected=%d target_v=%.2f cur_v=%.2f motor=%.1f servo=%.2f",
    static_cast<int>(detected), target_speed, g_current_speed_mps, motor_cmd, servo_hw);
}
