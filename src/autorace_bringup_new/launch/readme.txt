# 로봇 측에서 bringup을 한다 - 로봇은 토픽 발사대 개념으로 생각하면 됨,
# 코드 계산이나, 제어는 컴퓨터에서 쏘는 식으로 함,

# 실행방법
source /opt/ros/noetic/setup.bash
# (워크스페이스 빌드되어 있고, 해당 패키지가 catkin 패키지여야 합니다)
roslaunch autorace_bringup autorace_bringup.launch

# ------------------------------------------------------

위 3개의 launch를 통합
1. motor bringup
/root/autorace_kkk_ws/src/vesc/vesc_driver/launch/vesc_driver_node.launch

2. camera bringup
/root/autorace_kkk_ws/src/autorace_perception/launch/camera_calibration.launch


3. lidar bringup
/root/autorace_kkk_ws/src/rplidar_ros/launch/rplidar_s1.launch