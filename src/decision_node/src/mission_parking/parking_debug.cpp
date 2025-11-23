// 디버그용 단독 실행 엔트리: mission_parking의 init/step 호출
#include <ros/ros.h>

// mission_parking.cpp에서 제공
void mission_parking_init(ros::NodeHandle &nh, ros::NodeHandle &pnh);
void mission_parking_step();

int main(int argc, char **argv) {
  ros::init(argc, argv, "parking_mission_debug");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("[parking_main] starting parking mission (debug)");
  mission_parking_init(nh, pnh);

  ros::Rate rate(10.0);  // parking은 기본 10Hz 처리
  while (ros::ok()) {
    ros::spinOnce();
    mission_parking_step();
    rate.sleep();
  }
  return 0;
}
