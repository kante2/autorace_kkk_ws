/*

차선으로 부터 flag을 받아서, 차선 추종을 pid + pure pursuit으로 제어한다.

장애물의 회피 경로인 local path경로를 받으면 해당하는 회피 로직을 수행한다.

이후 다시 차선 추종

/

미션별로 나누기
메인이 토픽으로 제어
메인 -> 미션1 토픽주고 - 미션1 노드가 받아서 미션 수행 
 미션1노드가 미션 수행후 메인에 토픽날림

 메인 -> 미션2 토픽주고 ~~..


 미션 안에서는 차선을 인식하며 주행을 하다가,
 lidar 플래그가 날라오면 lidar_obstacle_avoid.cpp을 동작시키도록 함.
 이후 장애물 회피가 완료되면 플래그를 받아, 다시 차선 주행을 시킨다.

 

- 개발순서

1-tm1. pid + prue pursuit으로 기록한 path을 따라가도록 만든다.
1-tm2. 차선인식 및 곡률을 계산한다. (버드뷰 + 차선중심 완성)

2-tm1. 라이다 회피경로를 생성하고 rviz시각화, 그리고 장애물에 따른 회피를 수행(직선 회피만 고려)
2-tm2, 차선 따라가도록 제어 코드 완성. / 차선 바닥면 색 인식해서 퍼블리시 하도록 완성

3-tm1. (곡선구간 - 장애물회피 )곡선에서 라이다 회피 경로를 동적으로 반영하고, 이에 맞는 제어를 생성한다.(pid제어쪽?) (곡선 회피 고려)
3-tm2. (직선구간 - 차선과 장애물회피 ) 차선 주행중, 장애물에 따른 회피 후, 차선 주행모드로 복귀 완성.


- 중간
차선에 따른 주행과, 직선,곡선구간에서 모두 장애물 회피가 완성

4-tm1. 주차미션
4-tm2. 암실미션

5-tm1. 
5-tm2. 

*/
// src/autorace_decision/src/main_node.cpp
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "main_node");
  ros::NodeHandle nh;
  ROS_INFO("autorace_decision main_node started");
  ros::spin();
  return 0;
}
