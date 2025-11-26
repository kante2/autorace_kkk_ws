#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <cstdlib>
#include <optional>
#include <string>

namespace
{
// 토픽 이름
std::string g_parking_detected_topic;
std::string g_parking_goal_topic;
std::string g_motor_topic;
std::string g_bag_path;
std::string g_bag_lock_topic;

// 상태
ros::Subscriber g_sub_detected;
ros::Subscriber g_sub_goal;
ros::Publisher  g_pub_detected;
ros::Publisher  g_pub_goal;
ros::Publisher  g_pub_motor;
ros::Publisher  g_pub_bag_lock;

bool g_latest_detected = false;
ros::Time g_last_detect_time;
std::optional<geometry_msgs::PoseStamped> g_latest_goal;
double g_stop_motor_cmd = 0.0;

bool g_goal_in_range = false;
ros::Time g_goal_in_range_time;
bool g_bag_started = false;
double g_bag_delay_sec = 2.0;
double g_goal_x_min = -0.3;
double g_goal_x_max = 0.3;
bool g_bag_lock = false;  // bag 실행 중 파킹 미션 고정

void detectedCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (g_bag_lock) {
    // bag 실행 중에는 외부 감지 메시지로 상태를 덮어쓰지 않음
    return;
  }
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
  const double x = msg->pose.position.x;
  if (x >= g_goal_x_min && x <= g_goal_x_max) {
    g_goal_in_range = true;
    g_goal_in_range_time = ros::Time::now();
    g_bag_started = false;  // reset trigger for new in-range event
  }
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
  pnh.param<std::string>("bag_path", g_bag_path,
                         std::string("/root/autorace_kkk_ws/src/bagfiles/parking_command.bag"));
  pnh.param<std::string>("bag_lock_topic", g_bag_lock_topic,
                         std::string("/parking_bag_lock"));
  pnh.param<double>("parking_stop_motor_cmd", g_stop_motor_cmd, 0.0);
  pnh.param<double>("bag_delay_sec", g_bag_delay_sec, 2.0);
  pnh.param<double>("goal_x_min", g_goal_x_min, -0.3);
  pnh.param<double>("goal_x_max", g_goal_x_max, 0.3);

  ROS_INFO("[parking_mission] subscribe detected='%s', goal='%s'",
           ros::names::resolve(g_parking_detected_topic).c_str(),
           ros::names::resolve(g_parking_goal_topic).c_str());
  ROS_INFO("[parking_mission] republish detected='%s', goal='%s', motor='%s'(stop=%.1f)",
           ros::names::resolve(g_parking_detected_topic).c_str(),
           ros::names::resolve(g_parking_goal_topic).c_str(),
           ros::names::resolve(g_motor_topic).c_str(),
           g_stop_motor_cmd);
  ROS_INFO("[parking_mission] trigger bag when goal.x in [%.2f, %.2f], delay=%.1fs, bag='%s'",
           g_goal_x_min, g_goal_x_max, g_bag_delay_sec, g_bag_path.c_str());
  ROS_INFO("[parking_mission] publish bag_lock='%s'",
           ros::names::resolve(g_bag_lock_topic).c_str());

  g_sub_detected = nh.subscribe(g_parking_detected_topic, 1, detectedCallback);
  g_sub_goal = nh.subscribe(g_parking_goal_topic, 1, goalCallback);
  g_pub_detected = nh.advertise<std_msgs::Bool>(g_parking_detected_topic, 1);
  g_pub_goal = nh.advertise<geometry_msgs::PoseStamped>(g_parking_goal_topic, 1);
  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 1);
  g_pub_bag_lock = nh.advertise<std_msgs::Bool>(g_bag_lock_topic, 1, true);  // latched

  g_latest_detected = false;
  g_last_detect_time = ros::Time(0);
  g_latest_goal.reset();
  g_goal_in_range = false;
  g_bag_started = false;
  g_bag_lock = false;
}

void mission_parking_step()
{
  // 최신 goal/검출 캐시 재전송 (누락 방지용)
  if (g_latest_goal.has_value())
  {
    g_pub_goal.publish(*g_latest_goal);
  }

  std_msgs::Bool detected_msg;
  // bag 실행 중에는 파킹 미션을 유지하도록 강제로 true
  detected_msg.data = g_bag_lock ? true : g_latest_detected;
  g_pub_detected.publish(detected_msg);

  if (g_bag_lock || g_latest_detected)
  {
    std_msgs::Float64 motor_msg;
    motor_msg.data = g_stop_motor_cmd;
    g_pub_motor.publish(motor_msg);
  }

  // bag lock 상태를 퍼블리시 (latched로 유지)
  if (g_pub_bag_lock)
  {
    std_msgs::Bool lock_msg;
    lock_msg.data = g_bag_lock;
    g_pub_bag_lock.publish(lock_msg);
  }

  // goal x 범위 내면 정지 후 딜레이 뒤 rosbag 재생 (한 번만 트리거)
  if (g_goal_in_range && !g_bag_started)
  {
    const double dt = (ros::Time::now() - g_goal_in_range_time).toSec();
    if (dt >= g_bag_delay_sec)
    {
      const std::string cmd = "rosbag play " + g_bag_path + " &";
      int ret = std::system(cmd.c_str());
      g_bag_started = true;
      g_bag_lock = true;  // 파킹 상태 유지
      ROS_INFO("[parking_mission] goal in range -> starting bag: '%s' (ret=%d)", g_bag_path.c_str(), ret);
    }
  }

  ROS_INFO_THROTTLE(1.0,
                    "[parking_mission] detected=%d (cached %0.1f s ago)%s goal_in_range=%d bag_started=%d bag_lock=%d",
                    static_cast<int>(g_bag_lock ? true : g_latest_detected),
                    (ros::Time::now() - g_last_detect_time).toSec(),
                    g_latest_goal ? " goal republished" : "",
                    static_cast<int>(g_goal_in_range),
                    static_cast<int>(g_bag_started),
                    static_cast<int>(g_bag_lock));
}
