# autorace_decision 사용법 (요약)

## 구조
- `main_node.cpp`만 ROS 노드 엔트리포인트입니다. 모든 미션은 `mission_*_init()`, `mission_*_step()` 함수로 모듈화돼 있고, `main_node`가 호출합니다.
- `mission_*_toMain.cpp` : main_node 전용 모듈 코드(ros::init 없음).
- `mission_*.cpp` : 디버그/단독 실행용 노드(자체 `main()` 있음, 예: `mission_rotary/rotary_main.cpp`).

## 전체 실행 흐름
1) 센서/퍼셉션 런치  
```bash
roslaunch autorace_perception sensors_all.launch
```  
카메라+라이다 bringup, 라바콘/로터리 감지까지 포함.

2) 결정 노드 실행  
```bash
rosrun autorace_decision main_node
```  
각 미션 스테이트 머신이 순차로 실행됩니다(색→횡단보도→라바콘→턴사인→로터리→라바콘→라바콘→게이트).

## 단일 미션만 테스트하려면
- 차선 주행: `rosrun autorace_decision lane_center_controller`  
  입력 `/perception/center_point_px`, `/perception/curvature_center`
- 횡단보도: `rosrun autorace_decision mission_crosswalk_toMain`
- 라바콘 추종: `rosrun autorace_decision mission_labacorn_toMain`
- 게이트: `rosrun autorace_decision mission_gate_toMain`
- 회전표지판: `rosrun autorace_decision mission_AB_toMain`
- 로터리(메인 제어 버전): `rosrun autorace_decision mission_rotary_toMain`
- 색 구간 속도: `rosrun autorace_decision mission_color_toMain`

## 토픽/감지 요약
- 차선: `/perception/center_point_px`(PointStamped), `/perception/curvature_center`(Float64)
- 횡단보도: `/crosswalk_detected` (perception/crosswalk_node)
- 라바콘: `/labacorn_detected` (perception/cluster_detect_lidar)
- 로터리: `/rotary_detected` (perception/rotary_detect_lidar)
- 턴사인(YOLO): `/detect_left_sign`, `/detect_right_sign`
- 색 구간: `/perception/center_color_px` (x=0/1/2 none/red/blue)

## 미션별 디버깅 조합 예시
- 차선 주행  
  - 퍼셉션: `rosrun autorace_perception lane_center_node`  
  - 미션: `rosrun autorace_decision lane_center_controller`
- 횡단보도  
  - 퍼셉션: `rosrun autorace_perception crosswalk_node`  
  - 미션: `rosrun autorace_decision mission_crosswalk_toMain`
- 라바콘  
  - 퍼셉션: `rosrun autorace_perception cluster_detect_lidar`  
  - 미션: `rosrun autorace_decision mission_labacorn_toMain`
- 로터리  
  - 퍼셉션: `rosrun autorace_perception rotary_detect_lidar`  
  - 미션: `rosrun autorace_decision mission_rotary_toMain`
- 게이트  
  - 퍼셉션: (없음, 미션 내부에서 라이다 처리)  
  - 미션: `rosrun autorace_decision mission_gate_toMain`
- 턴사인(YOLO)  
  - 퍼셉션: `rosrun autorace_perception yolo_AB_sign_node.py`  
  - 미션: `rosrun autorace_decision mission_AB_toMain`
- 색 구간 속도  
  - 퍼셉션: `rosrun autorace_perception lane_color_node`  
  - 미션: `rosrun autorace_decision mission_color_toMain`

## 참고
- 파라미터는 각 노드의 `~` 네임스페이스에서 조정 가능합니다(토픽 이름, 속도/시간 상수 등).
- `main_node`는 1초 간격으로 현재 미션 스테이트를 로그로 출력합니다.
