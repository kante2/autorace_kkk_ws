// roatry_solo.cpp
// mission_rotary.cpp 로직을 단독으로 실행하는 간단한 래퍼 노드

#include <ros/ros.h>

// mission_rotary.cpp에서 제공
void mission_rotary_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_rotary_step();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rotary_solo");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  mission_rotary_init(nh, pnh);

  double loop_rate_hz = 30.0;
  pnh.param<double>("loop_rate_hz", loop_rate_hz, loop_rate_hz);
  ros::Rate rate(loop_rate_hz);

  ROS_INFO("[rotary_solo] start (rate=%.1f Hz)", loop_rate_hz);

  while (ros::ok()) {
    ros::spinOnce();
    mission_rotary_step();
    rate.sleep();
  }

  return 0;
}
