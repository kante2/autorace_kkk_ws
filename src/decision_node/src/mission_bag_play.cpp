// mission_bag_play.cpp
// bag_enable 토픽의 rising edge에서 rosbag play를 한 번 실행한다.

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <cstdlib>
#include <string>

namespace
{
std::string g_enable_topic;
std::string g_bag_path;
std::string g_bag_args;
bool g_last_enable = false;
bool g_busy = false;
ros::Time g_last_start;
double g_min_interval = 0.5;  // 초과 연속 트리거 방지

void enableCB(const std_msgs::Bool::ConstPtr& msg)
{
  const bool en = msg->data;
  const ros::Time now = ros::Time::now();

  // rising edge only
  if (en && !g_last_enable)
  {
    if (g_busy && (now - g_last_start).toSec() < g_min_interval)
    {
      ROS_WARN_THROTTLE(1.0, "[mission_bag_play] bag already started recently, skipping");
    }
    else
    {
      g_last_start = now;
      g_busy = true;
      const std::string cmd = "rosbag play " + g_bag_args + " " + g_bag_path + " &";
      int ret = std::system(cmd.c_str());
      ROS_INFO("[mission_bag_play] enable rising -> playing bag '%s' (ret=%d)", g_bag_path.c_str(), ret);
    }
  }

  g_last_enable = en;
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_bag_play");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<std::string>("enable_topic", g_enable_topic, std::string("/mission/bag_enable"));
  pnh.param<std::string>("bag_path", g_bag_path, std::string("/root/autorace_kkk_ws/src/bagfiles/parking_command.bag"));
  pnh.param<std::string>("bag_args", g_bag_args, std::string(""));
  pnh.param<double>("min_interval", g_min_interval, 0.5);

  ROS_INFO("[mission_bag_play] subscribe enable='%s'", ros::names::resolve(g_enable_topic).c_str());
  ROS_INFO("[mission_bag_play] will run: rosbag play %s %s", g_bag_args.c_str(), g_bag_path.c_str());

  ros::Subscriber sub = nh.subscribe(g_enable_topic, 1, enableCB);
  ros::spin();
  return 0;
}
