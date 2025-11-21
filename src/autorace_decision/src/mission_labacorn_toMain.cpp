#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

#include "autorace_control/controller_lane_logic.hpp"
#include "autorace_control/controller_obstacle_dbscan_logic.hpp"

// ======================================================================
//  lane_logic.hpp 에서 extern 으로 선언된 것들 "실제 정의"
//  (⚠ main_node + 다른 미션 파일에서 중복 정의 안 나게 나중에 정리 필요)
// ======================================================================

// 토픽 이름들
std::string g_topic_center_point;
std::string g_topic_dx_px;
std::string g_motor_topic;
std::string g_servo_topic;
std::string g_is_crosswalk_topic;
std::string g_topic_curvature_center;

// BEV 중심 x 픽셀
double g_bev_center_x_px = 320.0;

// 서보 스케일
double g_servo_center = 0.5;
double g_servo_min    = 0.0;
double g_servo_max    = 1.0;
double g_steer_sign   = -1.0;  // 파이썬 기본값

// Control 파라미터
double g_max_abs_dx_px = 83.0; // 100 -> 83
double g_dx_tolerance  = 3.0;
double g_steer_gain    = 1.5;    // 이 값이 center_curvature에 따라 실시간으로 변함
double g_alpha_ema     = 0.2;
double g_max_delta     = 0.08;

// 속도 계획
double g_base_speed_mps  = 7.0;
double g_min_speed_mps   = 0.8;
double g_speed_drop_gain = 0.5;

// 모터 스케일
double g_motor_min_cmd = 0.0;
double g_motor_max_cmd = 900.0;
double g_motor_gain    = 300.0;

// 타임아웃
double g_dx_timeout_sec = 1.0;

// 퍼블리셔
ros::Publisher g_pub_motor;
ros::Publisher g_pub_servo;

// 내부 상태
double    g_prev_steer       = 0.0; // 내부 steer_cmd (-1 ~ +1)
double    g_latest_steer_cmd = 0.0;
double    g_latest_speed_cmd = 0.0;
ros::Time g_last_cb_time;
bool      g_have_cb_time     = false;

// crosswalk 상태
bool      g_is_crosswalk            = false;   // 토픽에서 받은 flag
bool      g_crosswalk_timer_running = false;   // 7초 타이머 동작 여부
ros::Time g_crosswalk_start_time;             // 7초 타이머 시작 시각

// 곡률 범위 파라미터
double g_min_curv = 3e-4;    // 최소 곡률 (예: 직선에 가까움)
double g_max_curv = 1.5e-3;   // 최대 곡률 (예: 많이 휘어짐)

// PID
double g_kp_lat   = 1.5;
double g_ki_lat   = 0.0;
double g_kd_lat   = 0.0;
double g_int_sat  = 0.5;

// lane 쪽 구독자 (필요하면 사용)
ros::Subscriber g_sub_crosswalk;   // crosswalk 플래그만 쓰면 이것만 있어도 됨

// ======================================================================
//  obstacle_dbscan_logic.hpp 에서 extern 으로 선언된 것들 "실제 정의"
// ======================================================================

// 토픽 이름들 (LaserScan / Marker / Cloud)
std::string g_obs_scan_topic;
std::string g_obs_marker_topic;
std::string g_obs_cloud_topic;

// DBSCAN / ROI 파라미터
double g_obs_eps              = 0.2;
int    g_obs_min_samples      = 4;
double g_obs_range_min        = 0.13;
double g_obs_range_max        = 0.9;
double g_obs_front_min_deg    = 60.0;
double g_obs_front_max_deg    = 300.0;
double g_obs_lane_offset_y    = 0.12;

// yaw -> 조향/속도 파라미터
double g_obs_yaw_gain         = 0.8;
double g_obs_follow_speed_mps = 1.0;
double g_obs_min_speed_mps    = 0.7;

// 타임아웃
double g_obs_scan_timeout_sec = 0.5;

// 상태 변수 (scan / target)
bool      g_obs_have_scan_time   = false;
ros::Time g_obs_last_scan_time;
bool      g_obs_have_target      = false;
double    g_obs_latest_steer_cmd = 0.0;   // -1 ~ +1
double    g_obs_latest_speed_cmd = 0.0;   // m/s

// 시각화 퍼블리셔
ros::Publisher g_pub_obs_marker;
ros::Publisher g_pub_obs_cloud;

// obstacle scan 구독자
ros::Subscriber g_sub_obs_scan;


// ======================================================================
//  mission_labacorn: init / step
//  (main_node.cpp 에서 호출)
// ======================================================================

// 초기화: 토픽, 파라미터, pub/sub 세팅
void mission_labacorn_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[mission_labacorn] init start (obstacle DBSCAN + lane motor logic)");

  // 1) lane controller 파라미터 로드 (servo/motor 스케일, crosswalk 등)
  loadParams(pnh);

  // 2) obstacle DBSCAN 파라미터 로드
  loadParams_obstacle_dbscan(pnh);

  ROS_INFO("[mission_labacorn] subscribe scan='%s'",
           ros::names::resolve(g_obs_scan_topic).c_str());
  ROS_INFO("[mission_labacorn] subscribe is_crosswalk='%s'",
           ros::names::resolve(g_is_crosswalk_topic).c_str());
  ROS_INFO("[mission_labacorn] publish motor='%s', servo='%s'",
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str());
  ROS_INFO("[mission_labacorn] publish obs_marker='%s', obs_cloud='%s'",
           ros::names::resolve(g_obs_marker_topic).c_str(),
           ros::names::resolve(g_obs_cloud_topic).c_str());

  // --- Pub/Sub 설정 ---

  // Lidar scan → 장애물 DBSCAN 콜백
  g_sub_obs_scan =
      nh.subscribe(g_obs_scan_topic, 1, CB_obstacle_scan);

  // crosswalk 플래그 (lane_logic.cpp에 있는 CB_crosswalk 재사용)
  g_sub_crosswalk =
      nh.subscribe(g_is_crosswalk_topic, 10, CB_crosswalk);

  // 모터/서보 퍼블리셔 (lane_logic에서 사용)
  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_pub_servo = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);

  // 장애물 시각화 퍼블리셔
  g_pub_obs_marker =
      nh.advertise<visualization_msgs::Marker>(g_obs_marker_topic, 10);
  g_pub_obs_cloud =
      nh.advertise<sensor_msgs::PointCloud2>(g_obs_cloud_topic, 1);

  ROS_INFO("[mission_labacorn] init done");
}

// 한 스텝 실행: main_node의 while 루프 안에서 주기적으로 호출
void mission_labacorn_step()
{
  ros::Time now = ros::Time::now();

  // 1) Lidar scan timeout 체크
  bool have_scan = false;
  if (g_obs_have_scan_time)
  {
    double dt = (now - g_obs_last_scan_time).toSec();
    have_scan = (dt <= g_obs_scan_timeout_sec);
  }

  // 2) 기본 조향/속도는 장애물 DBSCAN 결과를 사용
  double steer_cmd = 0.0; // -1 ~ +1
  double speed_cmd = 0.0; // m/s

  if (have_scan && g_obs_have_target)
  {
    steer_cmd = g_obs_latest_steer_cmd;
    speed_cmd = g_obs_latest_speed_cmd;
  }
  else
  {
    // 스캔이 없거나 타겟이 없으면 일단 정지
    steer_cmd = 0.0;
    speed_cmd = 0.0;

    if (!have_scan)
    {
      ROS_INFO_THROTTLE(1.0,
        "[mission_labacorn] waiting LaserScan... (timeout)");
    }
    else
    {
      ROS_INFO_THROTTLE(1.0,
        "[mission_labacorn] have_scan but no obstacle target -> stop");
    }
  }

  // 3) crosswalk 7초 정지 로직 (lane_logic 쪽 함수 재사용)
  apply_crosswalk_hold(steer_cmd, speed_cmd, now);

  // 4) 조향/속도 → 서보/모터 명령으로 변환 & Publish
  publish_motor_commands(steer_cmd, speed_cmd);

  ROS_INFO_THROTTLE(0.5,
    "[mission_labacorn][step] have_scan=%d have_target=%d "
    "steer=%.3f v=%.2f",
    (int)have_scan, (int)g_obs_have_target,
    steer_cmd, speed_cmd);
}
