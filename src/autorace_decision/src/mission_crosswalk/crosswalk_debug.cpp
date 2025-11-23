// 디버그용 단독 실행 엔트리: mission_crosswalk의 init/step을 호출
#include <ros/ros.h>

// mission_crosswalk.cpp 에서 제공
void mission_crosswalk_init(ros::NodeHandle &nh, ros::NodeHandle &pnh);
void mission_crosswalk_step();

int main(int argc, char **argv) {
  ros::init(argc, argv, "crosswalk_mission_debug");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("[crosswalk_main] starting crosswalk mission (debug)");
  mission_crosswalk_init(nh, pnh);

  ros::Rate rate(30.0);  // 30 Hz
  while (ros::ok()) {
    ros::spinOnce();
    mission_crosswalk_step();
    rate.sleep();
  }
  return 0;
}
