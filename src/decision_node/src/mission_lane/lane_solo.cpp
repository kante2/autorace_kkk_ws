// lane_solo.cpp
// mission_lane.cpp의 로직을 단독으로 돌리는 간단한 래퍼 노드

#include <ros/ros.h>

// mission_lane.cpp에 구현되어 있음
void mission_lane_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_lane_step();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_solo");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  mission_lane_init(nh, pnh);

  double loop_rate_hz = 30.0;
  pnh.param<double>("loop_rate_hz", loop_rate_hz, loop_rate_hz);
  ros::Rate rate(loop_rate_hz);

  ROS_INFO("[lane_solo] start (rate=%.1f Hz)", loop_rate_hz);

  while (ros::ok()) {
    ros::spinOnce();
    mission_lane_step();
    rate.sleep();
  }

  return 0;
}
