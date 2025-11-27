// mission_rotary.cpp
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

namespace {
// 토픽 이름
std::string g_topic_rotary_detected;
std::string g_topic_motor;
std::string g_topic_servo;

// 속도/모터 파라미터
double g_cruise_speed_mps = 1.0;   // 감지 없을 때 기본 속도 (m/s)
double g_stop_speed_mps   = 0.0;   // 감지 시 속도 (보통 0)
double g_motor_min_cmd    = 0.0;
double g_motor_max_cmd    = 1000.0;
double g_motor_gain       = 3846.15; // m/s -> 모터 명령 (1000cmd = 0.26m/s)

// 조향 파라미터 (항상 센터로 유지)
double g_servo_center = 0.57;
double g_servo_min    = 0.0;
double g_servo_max    = 1.0;

// 상태
ros::Subscriber g_detect_sub;
ros::Publisher  g_motor_pub;
ros::Publisher  g_servo_pub;

bool g_rotary_detected = false;

inline double clamp(double x, double lo, double hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

void rotaryDetectedCB(const std_msgs::Bool::ConstPtr& msg) {
  g_rotary_detected = msg->data;
}

}  // namespace

// =====================================================
// main_node.cpp 에서 사용할 init / step 함수
// =====================================================

void mission_rotary_init(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  ROS_INFO("[mission_rotary] init (pure control: stop/go)");

  // 파라미터 로드
  pnh.param<std::string>("rotary_detected_topic",
                         g_topic_rotary_detected,
                         std::string("/rotary_detected"));
  pnh.param<std::string>("motor_topic",
                         g_topic_motor,
                         std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic",
                         g_topic_servo,
                         std::string("/commands/servo/position"));

  pnh.param<double>("cruise_speed_mps", g_cruise_speed_mps, 1.0);   // 감지 없을 때 속도
  pnh.param<double>("stop_speed_mps",   g_stop_speed_mps,   0.0);   // 감지 시 속도
  pnh.param<double>("motor_min_cmd",    g_motor_min_cmd,    0.0);
  pnh.param<double>("motor_max_cmd",    g_motor_max_cmd,    2000.0);
  pnh.param<double>("motor_gain",       g_motor_gain,       3846.15);

  pnh.param<double>("servo_center", g_servo_center, 0.5);
  pnh.param<double>("servo_min",    g_servo_min,    0.0);
  pnh.param<double>("servo_max",    g_servo_max,    1.0);

  ROS_INFO("[mission_rotary] subscribe detected='%s'",
           ros::names::resolve(g_topic_rotary_detected).c_str());
  ROS_INFO("[mission_rotary] publish motor='%s'",
           ros::names::resolve(g_topic_motor).c_str());
  ROS_INFO("[mission_rotary] publish servo='%s' (center=%.2f)",
           ros::names::resolve(g_topic_servo).c_str(), g_servo_center);

  // Pub/Sub 설정
  g_motor_pub  = nh.advertise<std_msgs::Float64>(g_topic_motor, 10);
  g_servo_pub  = nh.advertise<std_msgs::Float64>(g_topic_servo, 10);
  g_detect_sub = nh.subscribe(g_topic_rotary_detected, 1, rotaryDetectedCB);

  ROS_INFO("[mission_rotary] init done");
}

// main loop에서 호출
void mission_rotary_step() {
  // 순수하게 현재 detect 플래그만 보고 속도 결정
  const double target_speed_mps = g_rotary_detected ? g_stop_speed_mps
                                                    : g_cruise_speed_mps;

  double motor_cmd = g_motor_gain * target_speed_mps;
  motor_cmd = clamp(motor_cmd, g_motor_min_cmd, g_motor_max_cmd);

  // 서보는 항상 센터 유지
  double servo_hw = clamp(g_servo_center, g_servo_min, g_servo_max);

  std_msgs::Float64 motor_msg;
  std_msgs::Float64 servo_msg;
  motor_msg.data = motor_cmd;
  servo_msg.data = servo_hw;
  g_motor_pub.publish(motor_msg);
  g_servo_pub.publish(servo_msg);

  ROS_INFO_THROTTLE(0.5,
                    "[mission_rotary] detected=%d motor=%.1f speed=%.2f servo=%.2f",
                    static_cast<int>(g_rotary_detected),
                    motor_cmd,
                    target_speed_mps,
                    servo_hw);
}
