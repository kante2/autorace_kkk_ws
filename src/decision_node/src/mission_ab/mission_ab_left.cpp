// mission_ab_left.cpp
//  - Hardcoded left insertion maneuver for AB sign

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace
{
std::string g_motor_topic;
std::string g_servo_topic;
double g_duration_sec = 2.0;
double g_motor_cmd = 1200.0;   // forward speed command
double g_servo_left = 0.3;     // left turn servo position (0.0~1.0)
double g_servo_center = 0.5;   // center position to stop

ros::Publisher g_pub_motor;
ros::Publisher g_pub_servo;

bool g_started = false;
bool g_done = false;
ros::Time g_start_time;
}  // namespace

// Called from main_node init
void mission_ab_left_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  pnh.param<std::string>("motor_topic", g_motor_topic, std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic", g_servo_topic, std::string("/commands/servo/position"));
  pnh.param<double>("ab_left_duration_sec", g_duration_sec, 2.0);
  pnh.param<double>("ab_left_motor_cmd", g_motor_cmd, 1200.0);
  pnh.param<double>("ab_left_servo", g_servo_left, 0.3);
  pnh.param<double>("ab_servo_center", g_servo_center, 0.5);

  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 1);
  g_pub_servo = nh.advertise<std_msgs::Float64>(g_servo_topic, 1);

  g_started = false;
  g_done = false;
  g_start_time = ros::Time(0);

  ROS_INFO("[mission_ab_left] init motor='%s' servo='%s' duration=%.2fs motor=%.1f servo_left=%.2f",
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str(),
           g_duration_sec, g_motor_cmd, g_servo_left);
}

void mission_ab_left_step()
{
  if (g_done) {
    return;
  }

  ros::Time now = ros::Time::now();
  if (!g_started) {
    g_started = true;
    g_start_time = now;
  }

  // publish turning command
  std_msgs::Float64 motor_msg; motor_msg.data = g_motor_cmd;
  std_msgs::Float64 servo_msg; servo_msg.data = g_servo_left;
  g_pub_motor.publish(motor_msg);
  g_pub_servo.publish(servo_msg);

  double dt = (now - g_start_time).toSec();
  if (dt >= g_duration_sec) {
    // stop and mark done
    std_msgs::Float64 stop_motor; stop_motor.data = 0.0;
    std_msgs::Float64 center_servo; center_servo.data = g_servo_center;
    g_pub_motor.publish(stop_motor);
    g_pub_servo.publish(center_servo);
    g_done = true;
    ROS_INFO("[mission_ab_left] completed (%.2fs)", dt);
  }
}

bool mission_ab_left_done()
{
  return g_done;
}

void mission_ab_left_reset()
{
  g_started = false;
  g_done = false;
  g_start_time = ros::Time(0);
}
