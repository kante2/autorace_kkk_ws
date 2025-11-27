// labacorn_solo.cpp
// mission_labacorn.cpp 로직을 단독 실행하는 간단한 래퍼 노드

#include <ros/ros.h>

// mission_labacorn.cpp에서 제공
void mission_labacorn_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_labacorn_step();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "labacorn_solo");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  mission_labacorn_init(nh, pnh);

  double loop_rate_hz = 30.0;
  pnh.param<double>("loop_rate_hz", loop_rate_hz, loop_rate_hz);
  ros::Rate rate(loop_rate_hz);

  ROS_INFO("[labacorn_solo] start (rate=%.1f Hz)", loop_rate_hz);

  while (ros::ok()) {
    ros::spinOnce();
    mission_labacorn_step();
    rate.sleep();
  }

  return 0;
}
