<!-- 1. wheel_odo ekf을 먼저 킨다. 
roslaunch robot_bringup wheel_ekf.launch

2. gmapping을 실행한다.
roslaunch robot_bringup gmapping.launch

3. 맵이 그려졌으면, 맵을 저장한다.
rosrun map_server map_saver -f ~/maps/my_map
# => ~/maps/my_map.yaml, my_map.pgm 생성 -->