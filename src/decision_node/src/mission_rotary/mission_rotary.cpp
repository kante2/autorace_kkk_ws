// mission_rotary.cpp
#include <string>
#include <cstdlib>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

namespace {
// 토픽 이름
std::string g_topic_rotary_detected;
std::string g_topic_motor;
std::string g_topic_servo;
std::string g_topic_rosback_motor;
std::string g_topic_rosback_servo;
std::string g_rosback_play_command;
bool g_auto_play_rosback = true;

// 모터/서보 명령 클램프 (필요 시)
double g_motor_min_cmd = 0.0;
double g_motor_max_cmd = 1500.0;
double g_servo_min = -1.0;
double g_servo_max = 1.0;

// 상태
ros::Subscriber g_detect_sub;
ros::Subscriber g_rosback_motor_sub;
ros::Subscriber g_rosback_servo_sub;
ros::Publisher  g_motor_pub;
ros::Publisher  g_servo_pub;

bool g_rotary_detected = false;
double g_rosback_motor_cmd = 0.0;
double g_rosback_servo_cmd = 0.5;
bool g_have_rosback_motor = false;
bool g_have_rosback_servo = false;
bool g_force_stop_active = false;
bool g_rosback_started = false;

inline double clamp(double x, double lo, double hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

void rotaryDetectedCB(const std_msgs::Bool::ConstPtr& msg) {
  g_rotary_detected = msg->data;
}

void rosbackMotorCB(const std_msgs::Float64::ConstPtr& msg) {
  g_rosback_motor_cmd = msg->data;
  g_have_rosback_motor = true;
}

void rosbackServoCB(const std_msgs::Float64::ConstPtr& msg) {
  g_rosback_servo_cmd = msg->data;
  g_have_rosback_servo = true;
}

void startRosbackPlayerIfNeeded() {
  if (!g_auto_play_rosback || g_rosback_started) {
    return;
  }
  if (g_rosback_play_command.empty()) {
    ROS_WARN("[mission_rotary] auto-play enabled but rosback_play_command is empty");
    return;
  }

  std::string cmd = g_rosback_play_command + " &";
  int ret = std::system(cmd.c_str());
  if (ret != 0) {
    ROS_ERROR("[mission_rotary] failed to start rotary rosbag command: '%s' (ret=%d)",
              g_rosback_play_command.c_str(), ret);
  } else {
    g_rosback_started = true;
    ROS_INFO("[mission_rotary] started rotary rosbag: %s", g_rosback_play_command.c_str());
  }
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
  pnh.param<std::string>("rosback_motor_topic",
                         g_topic_rosback_motor,
                         std::string("/rotary_rosback/motor_speed"));
  pnh.param<std::string>("rosback_servo_topic",
                         g_topic_rosback_servo,
                         std::string("/rotary_rosback/servo_position"));
  pnh.param<std::string>("rosback_play_command",
                         g_rosback_play_command,
                         std::string("rosbag play /root/autorace_kkk_ws/src/decision_node/src/mission_bagfiles/rotary_mission.bag --clock"));
  pnh.param<bool>("auto_play_rosback", g_auto_play_rosback, true);

  pnh.param<double>("motor_min_cmd",    g_motor_min_cmd,    0.0);
  pnh.param<double>("motor_max_cmd",    g_motor_max_cmd,    1500.0);
  pnh.param<double>("servo_min",        g_servo_min,       -1.0);
  pnh.param<double>("servo_max",        g_servo_max,        1.0);

  ROS_INFO("[mission_rotary] subscribe detected='%s'",
           ros::names::resolve(g_topic_rotary_detected).c_str());
  ROS_INFO("[mission_rotary] subscribe rotary_rosback motor='%s', servo='%s'",
           ros::names::resolve(g_topic_rosback_motor).c_str(),
           ros::names::resolve(g_topic_rosback_servo).c_str());
  ROS_INFO("[mission_rotary] publish motor='%s'",
           ros::names::resolve(g_topic_motor).c_str());
  ROS_INFO("[mission_rotary] publish servo='%s'",
           ros::names::resolve(g_topic_servo).c_str());

  // Pub/Sub 설정
  g_motor_pub  = nh.advertise<std_msgs::Float64>(g_topic_motor, 10);
  g_servo_pub  = nh.advertise<std_msgs::Float64>(g_topic_servo, 10);
  g_detect_sub = nh.subscribe(g_topic_rotary_detected, 1, rotaryDetectedCB);
  g_rosback_motor_sub = nh.subscribe(g_topic_rosback_motor, 1, rosbackMotorCB);
  g_rosback_servo_sub = nh.subscribe(g_topic_rosback_servo, 1, rosbackServoCB);

  startRosbackPlayerIfNeeded();

  ROS_INFO("[mission_rotary] init done");
}

// main loop에서 호출
void mission_rotary_step() {
  if (!g_have_rosback_motor || !g_have_rosback_servo) {
    ROS_WARN_THROTTLE(1.0,
                      "[mission_rotary] waiting rotary_rosback cmds (motor=%d servo=%d)",
                      static_cast<int>(g_have_rosback_motor),
                      static_cast<int>(g_have_rosback_servo));
    return;
  }

  double motor_cmd = g_rotary_detected ? 0.0 : g_rosback_motor_cmd;
  double servo_hw = g_rosback_servo_cmd;

  if (g_rotary_detected) {
    if (!g_force_stop_active) {
      ROS_INFO("[mission_rotary] dynamic obstacle detected -> forcing stop");
      g_force_stop_active = true;
    }
  } else if (g_force_stop_active) {
    ROS_INFO("[mission_rotary] dynamic obstacle cleared -> replaying rotary_rosback cmds");
    g_force_stop_active = false;
  }

  motor_cmd = clamp(motor_cmd, g_motor_min_cmd, g_motor_max_cmd);
  servo_hw = clamp(servo_hw, g_servo_min, g_servo_max);

  std_msgs::Float64 motor_msg;
  std_msgs::Float64 servo_msg;
  motor_msg.data = motor_cmd;
  servo_msg.data = servo_hw;
  g_motor_pub.publish(motor_msg);
  g_servo_pub.publish(servo_msg);

  ROS_INFO_THROTTLE(0.5,
                    "[mission_rotary] detected=%d motor=%.1f servo=%.2f (rosback motor=%.1f servo=%.2f)",
                    static_cast<int>(g_rotary_detected),
                    motor_cmd,
                    servo_hw,
                    g_rosback_motor_cmd,
                    g_rosback_servo_cmd);
}
