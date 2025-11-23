#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <optional>
#include <string>

namespace
{
// 토픽 이름
std::string g_parking_detected_topic;
std::string g_parking_goal_topic;
std::string g_motor_topic;

// 상태
ros::Subscriber g_sub_detected;
ros::Subscriber g_sub_goal;
ros::Publisher  g_pub_detected;
ros::Publisher  g_pub_goal;
ros::Publisher  g_pub_motor;

bool g_latest_detected = false;
ros::Time g_last_detect_time;
std::optional<geometry_msgs::PoseStamped> g_latest_goal;
double g_stop_motor_cmd = 0.0;

void detectedCallback(const std_msgs::Bool::ConstPtr& msg)
{
  g_latest_detected = msg->data;
  g_last_detect_time = ros::Time::now();

  if (g_pub_detected)
  {
    g_pub_detected.publish(*msg);
  }
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  g_latest_goal = *msg;
  if (g_pub_goal)
  {
    g_pub_goal.publish(*msg);
  }
}
}  // namespace

// 외부(main_node)에서 호출
void mission_parking_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[parking_mission] init");

  pnh.param<std::string>("parking_detected_topic", g_parking_detected_topic,
                         std::string("/parking_detected"));
  pnh.param<std::string>("parking_goal_topic", g_parking_goal_topic,
                         std::string("/parking_goal"));
  pnh.param<std::string>("motor_topic", g_motor_topic,
                         std::string("/commands/motor/speed"));
  pnh.param<double>("parking_stop_motor_cmd", g_stop_motor_cmd, 0.0);

  ROS_INFO("[parking_mission] subscribe detected='%s', goal='%s'",
           ros::names::resolve(g_parking_detected_topic).c_str(),
           ros::names::resolve(g_parking_goal_topic).c_str());
  ROS_INFO("[parking_mission] republish detected='%s', goal='%s', motor='%s'(stop=%.1f)",
           ros::names::resolve(g_parking_detected_topic).c_str(),
           ros::names::resolve(g_parking_goal_topic).c_str(),
           ros::names::resolve(g_motor_topic).c_str(),
           g_stop_motor_cmd);

  g_sub_detected = nh.subscribe(g_parking_detected_topic, 1, detectedCallback);
  g_sub_goal = nh.subscribe(g_parking_goal_topic, 1, goalCallback);
  g_pub_detected = nh.advertise<std_msgs::Bool>(g_parking_detected_topic, 1);
  g_pub_goal = nh.advertise<geometry_msgs::PoseStamped>(g_parking_goal_topic, 1);
  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 1);

  g_latest_detected = false;
  g_last_detect_time = ros::Time(0);
  g_latest_goal.reset();
}

void mission_parking_step()
{
  // 최신 goal/검출 캐시 재전송 (누락 방지용)
  if (g_latest_goal.has_value())
  {
    g_pub_goal.publish(*g_latest_goal);
  }

  std_msgs::Bool detected_msg;
  detected_msg.data = g_latest_detected;
  g_pub_detected.publish(detected_msg);

  if (g_latest_detected)
  {
    std_msgs::Float64 motor_msg;
    motor_msg.data = g_stop_motor_cmd;
    g_pub_motor.publish(motor_msg);
  }

  ROS_INFO_THROTTLE(1.0,
                    "[parking_mission] detected=%d (cached %0.1f s ago)%s",
                    static_cast<int>(g_latest_detected),
                    (ros::Time::now() - g_last_detect_time).toSec(),
                    g_latest_goal ? " goal republished" : "");
}
