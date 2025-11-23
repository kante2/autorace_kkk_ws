#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>

#include <string>
#include <cmath>

// -------------------- 유틸 --------------------
// inline 이라서 헤더에 정의해도 ODR 문제 없음
inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- 전역 상태 (extern 선언) --------------------

// 토픽 이름들
extern std::string g_topic_center_point;
extern std::string g_topic_dx_px;   // 참고용(실제 구독 X)
extern std::string g_motor_topic;
extern std::string g_servo_topic;
extern std::string g_is_crosswalk_topic;
extern std::string g_topic_curvature_center;   // ★ center curvature 토픽 이름

// BEV 중심 x 픽셀
extern double g_bev_center_x_px;

// 서보 스케일
extern double g_servo_center;
extern double g_servo_min;
extern double g_servo_max;
extern double g_steer_sign;

// Control 파라미터
extern double g_max_abs_dx_px;
extern double g_dx_tolerance;
extern double g_steer_gain;
extern double g_alpha_ema;
extern double g_max_delta;

// 속도 계획
extern double g_base_speed_mps;
extern double g_min_speed_mps;
extern double g_speed_drop_gain;

// 모터 스케일
extern double g_motor_min_cmd;
extern double g_motor_max_cmd;
extern double g_motor_gain;

// PID 게인
extern double g_ki_lat;
extern double g_kd_lat;
extern double g_int_sat;

// 타임아웃
extern double g_dx_timeout_sec;

// 퍼블리셔
extern ros::Publisher g_pub_motor;
extern ros::Publisher g_pub_servo;

// 내부 상태
extern double    g_prev_steer;
extern double    g_latest_steer_cmd;
extern double    g_latest_speed_cmd;
extern ros::Time g_last_cb_time;
extern bool      g_have_cb_time;

// crosswalk 상태
extern bool      g_is_crosswalk;
extern bool      g_crosswalk_timer_running;
extern ros::Time g_crosswalk_start_time;

// 곡률 범위 파라미터
extern double g_min_curv;
extern double g_max_curv;


// -------------------- 함수 선언 --------------------

// 파라미터 로딩
void loadParams(ros::NodeHandle& pnh);

// dx 처리 로직
void compute_Dx(double dx);

// 콜백들
void CB_center(const geometry_msgs::PointStamped::ConstPtr& msg);
void CB_crosswalk(const std_msgs::Bool::ConstPtr& msg);
void CB_center_curvature(const std_msgs::Float32::ConstPtr& msg);

// crosswalk 7초 정지 보정
void apply_crosswalk_hold(double& steer_cmd, double& speed_cmd, const ros::Time& now);


// 모터 명령 퍼블리시
void publish_motor_commands(double steer_cmd, double speed_cmd);
