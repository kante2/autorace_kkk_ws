// 디버그용 단독 실행 엔트리: gate 미션만 단독 실행
#include <ros/ros.h>

// mission_gate.cpp에서 제공
void mission_gate_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_gate_step();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gate_mission_debug");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("[gate_debug] starting gate mission (debug)");
  mission_gate_init(nh, pnh);

  ros::Rate rate(30.0);  // 30 Hz
  while (ros::ok())
  {
    ros::spinOnce();
    mission_gate_step();
    rate.sleep();
  }
  return 0;
}
