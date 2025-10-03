# robot_bringup (ROS1 Noetic)

실물 로봇 브링업용 패키지 템플릿입니다. 다음을 제공합니다.
- **static TF**: `base_link` → 센서 프레임(laser, imu_link, camera_link)
- **동적 TF**: 입력 오도메트리(`/wheel_odom`)로부터 **odom → base_link** TF 방송
- **launch**: 센서 오프셋/프레임명을 인자로 조정 가능
- (옵션) `robot_state_publisher`로 URDF 기반 TF 발행

## 의존성
```bash
sudo apt install ros-noetic-roscpp ros-noetic-tf ros-noetic-tf2-ros                  ros-noetic-tf2-geometry-msgs ros-noetic-nav-msgs                  ros-noetic-robot-state-publisher ros-noetic-xacro
```

## 빌드
```bash
cd ~/autorace_kkk_ws
catkin_make
source devel/setup.bash
```

## 사용
```bash
roslaunch robot_bringup bringup.launch   odom_topic:=/wheel_odom   odom_frame:=odom   base_frame:=base_link   laser_xyz:="0.23 0.00 0.18" laser_rpy:="0 0 0"   imu_xyz:="0.00 0.00 0.10"   imu_rpy:="0 0 0"   cam_xyz:="0.12 0.03 0.18"   cam_rpy:="1.5708 0 1.5708"
```

- `map -> odom`은 **SLAM/Localization** 노드가 발행합니다(별도 런치).
- 센서 오프셋은 실측/캘리브레이션 값으로 교체하세요.

## 노드 설명
### `odom_tf_broadcaster`
- Sub: `~odom_topic` (default `/wheel_odom`) – `nav_msgs/Odometry`
- Pub TF: `odom_frame -> base_frame` (default `odom -> base_link`)
- (옵션) `/odom`으로 재배포

파라미터:
- `~odom_topic` (string): 입력 오도메트리 토픽
- `~odom_frame` (string): 부모 프레임 (기본 `odom`)
- `~base_frame` (string): 자식 프레임 (기본 `base_link`)
- `~republish_odom` (bool): `/odom` 재퍼블리시 여부

## 디버깅
- `rosrun tf view_frames`
- `rosrun tf tf_echo odom base_link`
- RViz에서 TF 확인

## 주의
- **중복 TF 금지**: 같은 parent→child를 두 노드가 발행하면 안 됩니다.
- 단위: meter, radian. 프레임명: REP-103 준수.
