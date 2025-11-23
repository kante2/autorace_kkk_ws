#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace
{
struct Point2D
{
  double x;
  double y;
};

// 토픽 이름
std::string g_detected_topic;
std::string g_markers_topic;
std::string g_motor_topic;
std::string g_servo_topic;

// 파라미터
double g_k_yaw = 0.8;
double g_follow_speed_mps = 1.0;
double g_servo_center = 0.5;
double g_servo_min = 0.0;
double g_servo_max = 1.0;
double g_steer_sign = -1.0;
double g_motor_min_cmd = 0.0;
double g_motor_max_cmd = 2000.0;
double g_motor_gain = 1500.0;
double g_marker_timeout_sec = 0.5;
double g_lane_offset_y = 0.12;

// 상태
ros::Subscriber g_sub_detected;
ros::Subscriber g_sub_markers;
ros::Publisher  g_pub_motor;
ros::Publisher  g_pub_servo;

bool g_detected = false;
std::vector<Point2D> g_centroids;
ros::Time g_last_marker_time;

inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

void detectedCB(const std_msgs::Bool::ConstPtr& msg)
{
  g_detected = msg->data;
}

void markersCB(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  g_centroids.clear();
  for (const auto& m : msg->markers)
  {
    Point2D p{m.pose.position.x, m.pose.position.y};
    g_centroids.push_back(p);
  }
  g_last_marker_time = ros::Time::now();
}

bool computeTarget(const std::vector<Point2D>& centroids, double& target_x, double& target_y)
{
  double left_x_sum = 0.0, left_y_sum = 0.0;
  double right_x_sum = 0.0, right_y_sum = 0.0;
  int left_cnt = 0, right_cnt = 0;

  for (const auto& p : centroids)
  {
    if (p.y > 0.0)
    {
      left_x_sum += p.x;
      left_y_sum += p.y;
      ++left_cnt;
    }
    else
    {
      right_x_sum += p.x;
      right_y_sum += p.y;
      ++right_cnt;
    }
  }

  if (left_cnt > 0 && right_cnt > 0)
  {
    double lx = left_x_sum / left_cnt;
    double ly = left_y_sum / left_cnt;
    double rx = right_x_sum / right_cnt;
    double ry = right_y_sum / right_cnt;
    int total = left_cnt + right_cnt;
    target_x = static_cast<double>(right_cnt) / total * lx +
               static_cast<double>(left_cnt) / total * rx;
    target_y = static_cast<double>(right_cnt) / total * ly +
               static_cast<double>(left_cnt) / total * ry;
    return true;
  }
  else if (left_cnt > 0)
  {
    target_x = left_x_sum / left_cnt;
    target_y = (left_y_sum / left_cnt) - g_lane_offset_y;
    return true;
  }
  else if (right_cnt > 0)
  {
    target_x = right_x_sum / right_cnt;
    target_y = (right_y_sum / right_cnt) + g_lane_offset_y;
    return true;
  }
  return false;
}
}  // namespace

// main_node에서 호출
void mission_labacorn_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[mission_labacorn] init (consume marker array)");

  pnh.param<std::string>("labacorn_detected_topic", g_detected_topic, std::string("/labacorn_detected"));
  pnh.param<std::string>("labacorn_markers_topic", g_markers_topic, std::string("/dbscan_lines"));
  pnh.param<std::string>("motor_topic", g_motor_topic, std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic", g_servo_topic, std::string("/commands/servo/position"));

  pnh.param<double>("yaw_gain", g_k_yaw, 0.8);
  pnh.param<double>("follow_speed_mps", g_follow_speed_mps, 1.0);
  pnh.param<double>("servo_center", g_servo_center, 0.5);
  pnh.param<double>("servo_min", g_servo_min, 0.0);
  pnh.param<double>("servo_max", g_servo_max, 1.0);
  pnh.param<double>("steer_sign", g_steer_sign, -1.0);
  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);
  pnh.param<double>("motor_max_cmd", g_motor_max_cmd, 2000.0);
  pnh.param<double>("motor_gain", g_motor_gain, 1500.0);
  pnh.param<double>("marker_timeout_sec", g_marker_timeout_sec, 0.5);
  pnh.param<double>("lane_offset_y", g_lane_offset_y, 0.12);

  ROS_INFO("[mission_labacorn] subscribe detected='%s', markers='%s'",
           ros::names::resolve(g_detected_topic).c_str(),
           ros::names::resolve(g_markers_topic).c_str());
  ROS_INFO("[mission_labacorn] publish motor='%s', servo='%s'",
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str());

  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_pub_servo = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);
  g_sub_detected = nh.subscribe(g_detected_topic, 1, detectedCB);
  g_sub_markers = nh.subscribe(g_markers_topic, 1, markersCB);

  g_detected = false;
  g_centroids.clear();
  g_last_marker_time = ros::Time(0);
}

void mission_labacorn_step()
{
  ros::Time now = ros::Time::now();
  bool fresh_markers = false;
  if (!g_centroids.empty() && !g_last_marker_time.isZero())
  {
    fresh_markers = (now - g_last_marker_time).toSec() <= g_marker_timeout_sec;
  }

  double motor_out = 0.0;
  double servo_out = g_servo_center;

  if (g_detected && fresh_markers)
  {
    double target_x = 0.0, target_y = 0.0;
    if (computeTarget(g_centroids, target_x, target_y))
    {
      double yaw = std::atan2(target_y, target_x);
      double steer_cmd = std::tanh(g_k_yaw * yaw);
      double speed_cmd = g_follow_speed_mps;

      double steer_norm = clamp(steer_cmd, -1.0, 1.0);
      steer_norm *= (-g_steer_sign);
      double servo_range = std::min(g_servo_center - g_servo_min,
                                    g_servo_max - g_servo_center);
      servo_out = clamp(g_servo_center + steer_norm * servo_range,
                        g_servo_min, g_servo_max);

      motor_out = clamp(g_motor_gain * speed_cmd, g_motor_min_cmd, g_motor_max_cmd);
    }
  }

  std_msgs::Float64 motor_msg; motor_msg.data = motor_out;
  std_msgs::Float64 servo_msg; servo_msg.data = servo_out;
  g_pub_motor.publish(motor_msg);
  g_pub_servo.publish(servo_msg);

  ROS_INFO_THROTTLE(0.5,
                    "[mission_labacorn] detected=%d fresh_markers=%d motor=%.1f servo=%.3f (centroids=%zu)",
                    static_cast<int>(g_detected),
                    static_cast<int>(fresh_markers),
                    motor_out,
                    servo_out,
                    g_centroids.size());
}
