// ab_left_debug.cpp
// 단독 실행으로 mission_ab_left 로직을 디버깅하기 위한 노드

#include <ros/ros.h>

// main_node.cpp 에서 사용하는 인터페이스 선언
void mission_ab_left_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_ab_left_step();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ab_left_debug");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("[ab_left_debug] init");
  mission_ab_left_init(nh, pnh);

  ros::Rate rate(20.0);  // 디버깅 주기 20 Hz
  while (ros::ok())
  {
    ros::spinOnce();
    mission_ab_left_step();
    rate.sleep();
  }

  return 0;
}
