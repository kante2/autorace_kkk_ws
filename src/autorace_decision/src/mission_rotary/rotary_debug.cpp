// 디버그용 단독 실행 엔트리: mission_rotary의 init/step을 호출
#include <ros/ros.h>

// mission_rotary.cpp에서 제공
void mission_rotary_init(ros::NodeHandle &nh, ros::NodeHandle &pnh);
void mission_rotary_step();

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotary_mission_debug");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("[rotary_main] starting rotary mission (debug)");
  mission_rotary_init(nh, pnh);

  ros::Rate rate(30.0);  // 30 Hz
  while (ros::ok()) {
    ros::spinOnce();
    mission_rotary_step();
    rate.sleep();
  }
  return 0;
}
