// mission_rotary.cpp
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

namespace {
// 토픽 이름
std::string g_topic_rotary_detected;
std::string g_topic_motor;

// 속도/모터 파라미터
double g_cruise_speed_mps = 1.0;   // 감지 없을 때 기본 속도
double g_stop_speed_mps   = 0.0;   // 감지 시 정지
double g_motor_min_cmd = 0.0;
double g_motor_max_cmd = 2000.0;
double g_motor_gain = 1500.0;      // m/s -> 모터 명령

// 상태
ros::Subscriber g_detect_sub;
ros::Publisher g_motor_pub;

bool g_rotary_detected = false;
ros::Time g_last_detect_time;
double g_detect_timeout_sec = 0.5;

inline double clamp(double x, double lo, double hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

void rotaryDetectedCB(const std_msgs::Bool::ConstPtr& msg) {
  g_rotary_detected = msg->data;
  g_last_detect_time = ros::Time::now();
}
}  // namespace

// =====================================================
// main_node.cpp 에서 사용할 init / step 함수
// =====================================================

void mission_rotary_init(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  ROS_INFO("[mission_rotary] init");

  // 파라미터 로드
  pnh.param<std::string>("rotary_detected_topic", g_topic_rotary_detected, std::string("/rotary_detected"));
  pnh.param<std::string>("motor_topic", g_topic_motor, std::string("/commands/motor/speed"));

  pnh.param<double>("cruise_speed_mps", g_cruise_speed_mps, 1.0);
  pnh.param<double>("stop_speed_mps", g_stop_speed_mps, 0.0);

  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);
  pnh.param<double>("motor_max_cmd", g_motor_max_cmd, 2000.0);
  pnh.param<double>("motor_gain", g_motor_gain, 1500.0);
  pnh.param<double>("detect_timeout_sec", g_detect_timeout_sec, 0.5);

  ROS_INFO("[mission_rotary] subscribe detected='%s'",
           ros::names::resolve(g_topic_rotary_detected).c_str());
  ROS_INFO("[mission_rotary] publish motor='%s'",
           ros::names::resolve(g_topic_motor).c_str());

  // Pub/Sub 설정
  g_motor_pub = nh.advertise<std_msgs::Float64>(g_topic_motor, 10);
  g_detect_sub = nh.subscribe(g_topic_rotary_detected, 1, rotaryDetectedCB);
  g_last_detect_time = ros::Time(0);

  ROS_INFO("[mission_rotary] init done");
}

// main loop에서 호출
void mission_rotary_step() {
  bool detected = false;
  if (!g_last_detect_time.isZero()) {
    double dt = (ros::Time::now() - g_last_detect_time).toSec();
    detected = (dt <= g_detect_timeout_sec) && g_rotary_detected;
  }

  double target_speed = detected ? g_stop_speed_mps : g_cruise_speed_mps;
  double motor_cmd = clamp(g_motor_gain * target_speed, g_motor_min_cmd, g_motor_max_cmd);

  std_msgs::Float64 motor_msg;
  motor_msg.data = motor_cmd;
  g_motor_pub.publish(motor_msg);

  ROS_INFO_THROTTLE(0.5, "[mission_rotary] detected=%d motor=%.1f",
                    static_cast<int>(detected), motor_cmd);
}
