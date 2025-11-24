// mission_color_toMain.cpp
#include <ros/ros.h>
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
static std::string g_motor_topic;
static std::string g_servo_topic;

// 서보 스케일
static double g_servo_center = 0.5;
static double g_servo_min    = 0.0;
static double g_servo_max    = 1.0;
static double g_steer_sign   = -1.0;

// 모터 스케일
static double g_motor_min_cmd = 0.0;
static double g_motor_max_cmd = 2000.0;

// 색에 따른 모터 명령
static double g_red_motor_cmd  = 900.0;   // 빨강: 느리게
static double g_blue_motor_cmd = 1300.0;  // 파랑: 빠르게

// 퍼블리셔
static ros::Publisher g_pub_motor;
static ros::Publisher g_pub_servo;

// -------------------- 초기화 함수 --------------------
void mission_color_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[color_mission] mission_color_init()");

  // --- 파라미터 로드 ---
  pnh.param<std::string>("motor_topic", g_motor_topic,
                         std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic", g_servo_topic,
                         std::string("/commands/servo/position"));

  // 서보 스케일
  pnh.param<double>("servo_center", g_servo_center, 0.5);
  pnh.param<double>("servo_min",    g_servo_min,    0.0);
  pnh.param<double>("servo_max",    g_servo_max,    1.0);
  pnh.param<double>("steer_sign",   g_steer_sign,  -1.0);

  // 모터 스케일
  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);
  pnh.param<double>("motor_max_cmd", g_motor_max_cmd, 2000.0);

  // 색별 모터 명령
  pnh.param<double>("red_motor_cmd",  g_red_motor_cmd,  900.0);
  pnh.param<double>("blue_motor_cmd", g_blue_motor_cmd, 1300.0);

  ROS_INFO("[color_mission] publish motor='%s', servo='%s'",
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str());
  ROS_INFO("[color_mission] red_motor_cmd=%.1f, blue_motor_cmd=%.1f",
           g_red_motor_cmd, g_blue_motor_cmd);

  // --- Pub ---
  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_pub_servo = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);

  ROS_INFO("[color_mission] mission_color_init done");
}

// -------------------- 한 스텝 실행 --------------------
// color_code: 0=none, 1=red, 2=blue
void mission_color_step(int color_code)
{
  // 기본: 정지 + 직진 조향
  double steer_cmd = 0.0;
  double motor_cmd = 0.0;

  if (color_code == 1)
  {
    // 빨강: 느리게(900)
    motor_cmd = g_red_motor_cmd;
    ROS_INFO_THROTTLE(1.0,
      "[color_mission] RED zone -> motor=%.1f", motor_cmd);
  }
  else if (color_code == 2)
  {
    // 파랑: 빠르게(1300)
    motor_cmd = g_blue_motor_cmd;
    ROS_INFO_THROTTLE(1.0,
      "[color_mission] BLUE zone -> motor=%.1f", motor_cmd);
  }
  else
  {
    // 색이 없으면 정지 (원하면 여기서 lane 속도를 그대로 유지하도록 바꿀 수도 있음)
    motor_cmd = 0.0;
    ROS_INFO_THROTTLE(1.0,
      "[color_mission] color_code=0 -> stop");
  }

  // 모터 클램프
  motor_cmd = clamp(motor_cmd, g_motor_min_cmd, g_motor_max_cmd);

  // 조향: 항상 직진
  double steer_norm = clamp(steer_cmd, -1.0, 1.0);
  steer_norm *= (-g_steer_sign);

  double servo_range = std::min(g_servo_center - g_servo_min,
                                g_servo_max - g_servo_center);
  double servo_hw = g_servo_center + steer_norm * servo_range;
  servo_hw = clamp(servo_hw, g_servo_min, g_servo_max);

  // 퍼블리시
  std_msgs::Float64 motor_msg;
  std_msgs::Float64 servo_msg;
  motor_msg.data = motor_cmd;
  servo_msg.data = servo_hw;

  g_pub_motor.publish(motor_msg);
  g_pub_servo.publish(servo_msg);

  ROS_INFO_THROTTLE(0.5,
    "[color_mission][loop] color_code=%d steer=%.3f servo=%.3f motor=%.1f",
    color_code, steer_cmd, servo_hw, motor_cmd);
}
