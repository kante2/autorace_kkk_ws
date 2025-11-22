(1) main에서 쓰는 구조 (mission_*_toMain.cpp)

이 파일들은 하나의 실행파일(main_node) 안에 전부 같이 링크됨

예: main_node.cpp + mission_lane_toMain.cpp + mission_labacorn_toMain.cpp + mission_gate_toMain.cpp

​<autorace_perception>
lidar ---------------------->>> 클러스터링 결과
camera --------------------->>> 차선인식(곡률, 차선중앙과의 dx)/ crosswalk 인지결과 

<autorace_decision>
perception의 결과를 cb하고, 미션에 맞는 플래그를 true로 발동시켜,
각 미션 (예 mission_*_toMain())을 호출하는 식으로 동작한다.

참고로, toMain이 없는 대상은 단독 디버깅을 위해 남겨놓은 구조로, perception을 실행후, rosrun으로 동작시켜볼 수 있다.
