// mission_ab_left.cpp
//  - AB 구간: 왼쪽 차선만 추종 (좌측으로 오프셋) 후 8초 경과 시 정지

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>

#include <cmath>
#include <string>

namespace
{
// -------------------- 유틸 --------------------
inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- 파라미터/토픽 --------------------
static std::string g_topic_center_point;
static std::string g_motor_topic;
static std::string g_servo_topic;

static double g_bev_center_x_px  = 320.0;
static double g_target_offset_px = -80.0;  // 왼쪽 차선 쪽으로 당김 (왼쪽 음수)

static double g_servo_center = 0.5;
static double g_servo_min    = 0.0;
static double g_servo_max    = 1.0;
static double g_steer_sign   = -1.0;

static double g_max_abs_dx_px = 80.0;
static double g_dx_tolerance  = 5.0;
static double g_steer_gain    = 1.2;
static double g_alpha_ema     = 0.25;
static double g_max_delta     = 0.08;

static double g_base_speed_mps  = 6.0;
static double g_min_speed_mps   = 0.5;
static double g_speed_drop_gain = 0.5;

static double g_motor_min_cmd = 0.0;
static double g_motor_max_cmd = 900.0;
static double g_motor_gain    = 300.0;

static double g_dx_timeout_sec   = 1.0;
static double g_run_duration_sec = 8.0;  // 기본 8초

// -------------------- Pub/Sub --------------------
static ros::Publisher  g_pub_motor;
static ros::Publisher  g_pub_servo;
static ros::Subscriber g_center_sub;

// -------------------- 내부 상태 --------------------
static double    g_prev_steer        = 0.0;
static double    g_latest_steer_cmd  = 0.0;
static double    g_latest_speed_cmd  = 0.0;
static ros::Time g_last_cb_time;
static bool      g_have_cb_time = false;

static bool      g_started = false;
static bool      g_done = false;
static ros::Time g_start_time;

// -------------------- 내부 함수 --------------------
static void processDx(double dx)
{
  double err_norm = 0.0;
  if (std::fabs(dx) > g_dx_tolerance) {
    err_norm = clamp(dx / g_max_abs_dx_px, -1.0, 1.0);
  }

  double steer_raw = std::tanh(g_steer_gain * err_norm);
  double steer_smooth = g_alpha_ema * steer_raw + (1.0 - g_alpha_ema) * g_prev_steer;

  double delta = clamp(steer_smooth - g_prev_steer, -g_max_delta, g_max_delta);
  double steer_cmd = g_prev_steer + delta;
  g_prev_steer = steer_cmd;

  double speed_cmd = clamp(g_base_speed_mps - g_speed_drop_gain * std::fabs(err_norm),
                           g_min_speed_mps, g_base_speed_mps);

  g_latest_steer_cmd = steer_cmd;  // -1 ~ +1
  g_latest_speed_cmd = speed_cmd;  // m/s

  ROS_INFO_THROTTLE(0.5,
    "[ab_left] dx=%.1fpx err=%.3f steer=%.3f speed=%.2f",
    dx, err_norm, steer_cmd, speed_cmd);
}

static void centerCB(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  g_last_cb_time = ros::Time::now();
  g_have_cb_time = true;

  double cx = msg->point.x;
  double dx = cx - (g_bev_center_x_px + g_target_offset_px);

  processDx(dx);
}

static void publishCommands(double motor, double servo)
{
  std_msgs::Float64 m; m.data = motor;
  std_msgs::Float64 s; s.data = servo;
  g_pub_motor.publish(m);
  g_pub_servo.publish(s);
}
}  // namespace

// =====================================================
// main_node.cpp 에서 호출
// =====================================================

void mission_ab_left_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  pnh.param<std::string>("topic_center_point",
                         g_topic_center_point,
                         std::string("/perception/center_point_px"));
  pnh.param<std::string>("motor_topic", g_motor_topic,
                         std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic", g_servo_topic,
                         std::string("/commands/servo/position"));

  pnh.param<double>("bev_center_x_px", g_bev_center_x_px, 320.0);
  pnh.param<double>("target_offset_px_left", g_target_offset_px, -80.0);

  pnh.param<double>("servo_center", g_servo_center, 0.5);
  pnh.param<double>("servo_min",    g_servo_min,    0.0);
  pnh.param<double>("servo_max",    g_servo_max,    1.0);
  pnh.param<double>("steer_sign",   g_steer_sign,  -1.0);

  pnh.param<double>("max_abs_dx_px", g_max_abs_dx_px, 80.0);
  pnh.param<double>("dx_tolerance",  g_dx_tolerance,  5.0);
  pnh.param<double>("steer_gain",    g_steer_gain,    1.2);
  pnh.param<double>("steer_smoothing_alpha", g_alpha_ema, 0.25);
  pnh.param<double>("max_steer_delta_per_cycle", g_max_delta, 0.08);

  pnh.param<double>("base_speed_mps",  g_base_speed_mps,  6.0);
  pnh.param<double>("min_speed_mps",   g_min_speed_mps,   0.5);
  pnh.param<double>("speed_drop_gain", g_speed_drop_gain, 0.5);

  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);
  pnh.param<double>("motor_max_cmd", g_motor_max_cmd, 900.0);
  pnh.param<double>("motor_gain",    g_motor_gain,    300.0);

  pnh.param<double>("dx_timeout_sec", g_dx_timeout_sec, 1.0);
  pnh.param<double>("ab_left_run_duration_sec", g_run_duration_sec, 8.0);

  g_center_sub = nh.subscribe(g_topic_center_point, 20, centerCB);
  g_pub_motor  = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_pub_servo  = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);

  g_started = false;
  g_done = false;
  g_prev_steer = 0.0;
  g_have_cb_time = false;
  g_start_time = ros::Time(0);

  ROS_INFO("[ab_left] init: center='%s' motor='%s' servo='%s' offset=%.1fpx duration=%.1fs",
           ros::names::resolve(g_topic_center_point).c_str(),
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str(),
           g_target_offset_px, g_run_duration_sec);
}

void mission_ab_left_step()
{
  if (g_done) {
    return;
  }

  const ros::Time now = ros::Time::now();
  if (!g_started) {
    g_started = true;
    g_start_time = now;
  }

  bool have_dx = false;
  if (g_have_cb_time) {
    double dt = (now - g_last_cb_time).toSec();
    have_dx = (dt <= g_dx_timeout_sec);
  }

  double steer_cmd = 0.0;
  double speed_cmd = 0.0;

  if (have_dx) {
    steer_cmd = g_latest_steer_cmd;
    speed_cmd = g_latest_speed_cmd;
  }

  double t = (now - g_start_time).toSec();
  if (t >= g_run_duration_sec) {
    publishCommands(0.0, g_servo_center);
    g_done = true;
    ROS_INFO("[ab_left] done (t=%.2fs)", t);
    return;
  }

  double steer_norm = clamp(steer_cmd, -1.0, 1.0);
  steer_norm *= (-g_steer_sign);
  double servo_range = std::min(g_servo_center - g_servo_min,
                                g_servo_max - g_servo_center);
  double servo_hw = g_servo_center + steer_norm * servo_range;
  servo_hw = clamp(servo_hw, g_servo_min, g_servo_max);

  double motor_cmd = g_motor_gain * speed_cmd;
  motor_cmd = clamp(motor_cmd, g_motor_min_cmd, g_motor_max_cmd);

  publishCommands(motor_cmd, servo_hw);
}

bool mission_ab_left_done()
{
  return g_done;
}

void mission_ab_left_reset()
{
  g_started = false;
  g_done = false;
  g_prev_steer = 0.0;
  g_have_cb_time = false;
  g_start_time = ros::Time(0);
}

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>

#include <cmath>
#include <string>

// -------------------- 유틸 --------------------
inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- 전역 상태 --------------------

// 토픽 이름들
static std::string g_topic_left_center_point;   // 왼쪽 차선 중심
static std::string g_topic_dx_px;               // 참고용(실제 구독 X)
static std::string g_motor_topic;
static std::string g_servo_topic;
static std::string g_is_crosswalk_topic;
static std::string g_topic_curvature_center;
static std::string g_topic_center_color;

// BEV 중심 x 픽셀 (왼쪽 차선 기준으로 튜닝)
static double g_bev_center_x_px = 320.0;

// 서보 스케일
static double g_servo_center = 0.5;
static double g_servo_min    = 0.0;
static double g_servo_max    = 1.0;
static double g_steer_sign   = -1.0;

// Control 파라미터
static double g_max_abs_dx_px   = 83.0;
static double g_dx_tolerance    = 3.0;
static double g_steer_gain_base = 1.5;   // 곡률 반영 전 기본 gain
static double g_steer_gain_min  = 0.8;   // 직선 부근에서 최소 gain
static double g_steer_gain_max  = 2.0;   // 급커브에서 최대 gain
static double g_steer_gain      = 1.5;   // 실제 사용 gain (매 프레임 갱신)
static double g_alpha_ema       = 0.2;
static double g_max_delta       = 0.08;

// 속도 계획
static double g_base_speed_mps  = 7.0;
static double g_min_speed_mps   = 0.8;
static double g_speed_drop_gain = 0.5;

// 모터 스케일
static double g_motor_min_cmd = 0.0;
static double g_motor_max_cmd = 1200.0;
static double g_motor_gain    = 500.0;

// 타임아웃
static double g_dx_timeout_sec = 1.0;

// 퍼블리셔
static ros::Publisher g_pub_motor;
static ros::Publisher g_pub_servo;

// 서브스크라이버
static ros::Subscriber g_left_center_sub;
static ros::Subscriber g_center_color_sub;
static ros::Subscriber g_crosswalk_sub;
static ros::Subscriber g_curvature_sub;

// 내부 상태
static double   g_prev_steer       = 0.0;
static double   g_latest_steer_cmd = 0.0;
static double   g_latest_speed_cmd = 0.0;
static ros::Time g_last_cb_time;
static bool      g_have_cb_time    = false;
static bool      g_follow_running  = false;
static ros::Time g_follow_start;
static double    g_follow_duration_sec = 8.0;
static bool      g_sequence_done   = false;

// crosswalk 상태
static bool      g_is_crosswalk            = false;
static bool      g_crosswalk_timer_running = false;
static ros::Time g_crosswalk_start_time;

// 곡률 범위 파라미터
static double g_min_curv = 3e-6;
static double g_max_curv = 1e-3;

// PID 파라미터
static double g_pid_kp     = 0.02;
static double g_pid_ki     = 0.0;
static double g_pid_kd     = 0.001;
static double g_pid_weight = 0.3;

// PID 내부 상태
static double    g_pid_integral   = 0.0;
static double    g_pid_prev_error = 0.0;
static ros::Time g_pid_prev_time;
static bool      g_pid_has_prev_t = false;

// 색상 정보
static int g_lane_color_code = 0;   // 0 none, 1 red, 2 blue

// dx 스파이크 필터
static double g_prev_dx     = 0.0;
static bool   g_has_prev_dx = false;

// -------------------- PID 한 스텝 계산 --------------------
static double runPID(double error)
{
  ros::Time now = ros::Time::now();
  double dt = 0.0;

  if (g_pid_has_prev_t) {
    dt = (now - g_pid_prev_time).toSec();
  }
  g_pid_prev_time = now;
  g_pid_has_prev_t = true;

  double P = g_pid_kp * error;

  g_pid_integral += error * dt;
  double I = g_pid_ki * g_pid_integral;

  double D = 0.0;
  if (dt > 1e-4) {
    double diff = (error - g_pid_prev_error) / dt;
    D = g_pid_kd * diff;
  }
  g_pid_prev_error = error;

  return P + I + D;  // px 단위 raw 출력
}

// -------------------- dx 처리 로직 --------------------
static void processDx(double dx)
{
  double err_norm = 0.0;

  if (std::fabs(dx) <= g_dx_tolerance) {
    err_norm = 0.0;
  } else {
    err_norm = clamp(dx / g_max_abs_dx_px, -1.0, 1.0);
  }

  // -1 ~ +1 사이 조향 명령 (tanh 비선형)
  double steer_raw_base = std::tanh(g_steer_gain * err_norm);

  // PID 보정 (dx를 직접 에러로 사용)
  double u_pid   = runPID(dx);  // px 단위
  double steer_pid = clamp(u_pid / g_max_abs_dx_px, -1.0, 1.0);

  const double w_base = 1.0;
  double steer_raw = w_base * steer_raw_base + g_pid_weight * steer_pid;
  steer_raw = clamp(steer_raw, -1.0, 1.0);

  // EMA 스무딩
  double steer_smooth = g_alpha_ema * steer_raw + (1.0 - g_alpha_ema) * g_prev_steer;

  // 한 step당 변화 제한
  double delta = clamp(steer_smooth - g_prev_steer, -g_max_delta, g_max_delta);
  double steer_cmd = g_prev_steer + delta;
  g_prev_steer = steer_cmd;

  // 속도: 오차 클수록 줄이기
  double speed_cmd = clamp(g_base_speed_mps - g_speed_drop_gain * std::fabs(err_norm),
                           g_min_speed_mps, g_base_speed_mps);

  // 색상 기반 속도 조정
  if (g_lane_color_code == 1) {
    // red
    speed_cmd *= 0.5;
  } else if (g_lane_color_code == 2) {
    // blue
    speed_cmd = g_base_speed_mps;
  }

  g_latest_steer_cmd = steer_cmd;   // -1 ~ +1
  g_latest_speed_cmd = speed_cmd;   // m/s

  ROS_INFO_THROTTLE(0.5,
    "[lane_AB_left][CB+PID] dx=%.1fpx errN=%.3f steer=%.3f v=%.2f (steer_gain=%.3f, Kp=%.4f, Ki=%.4f, Kd=%.4f, color=%d)",
    dx, err_norm, steer_cmd, speed_cmd,
    g_steer_gain, g_pid_kp, g_pid_ki, g_pid_kd, g_lane_color_code);
}

// -------------------- 콜백: 왼쪽 차선 중심 --------------------
static void leftCenterCB(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  // 타임아웃 체크용 시간 저장
  g_last_cb_time = ros::Time::now();
  g_have_cb_time = true;

  double cx = msg->point.x;
  double dx_raw = cx - g_bev_center_x_px;   // 오른쪽이 +, 왼쪽이 -

  // 급점프 프레임 차단
  double dx = dx_raw;
  const double spike_thresh = 80.0; // px 기준
  if (g_has_prev_dx && std::fabs(dx_raw - g_prev_dx) > spike_thresh) {
    dx = g_prev_dx;   // 급점프 프레임 완전 무시
  }
  g_prev_dx = dx;
  g_has_prev_dx = true;

  processDx(dx);
}

// -------------------- 콜백: is_crosswalk --------------------
static void crosswalkCB(const std_msgs::Bool::ConstPtr& msg)
{
  g_is_crosswalk = msg->data;
}

// -------------------- 콜백: center_color --------------------
static void centerColorCB(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  g_lane_color_code = static_cast<int>(msg->point.x); // 0,1,2(red/blue)
}

// -------------------- 콜백: curvature_center --------------------
static void curvatureCenterCB(const std_msgs::Float32::ConstPtr& msg)
{
  double curv = msg->data; // 부호는 좌/우 구분, 여기서는 크기만 사용
  double k = std::fabs(curv);

  if (g_max_curv <= g_min_curv) {
    ROS_WARN_THROTTLE(1.0,
      "[lane_AB_left] invalid curvature range (min >= max), use base gain");
    g_steer_gain = g_steer_gain_base;
    return;
  }

  // 1) 곡률 크기 클램프
  double k_clamped = clamp(k, g_min_curv, g_max_curv);

  // 2) 0~1 노멀라이즈
  double t = (k_clamped - g_min_curv) / (g_max_curv - g_min_curv);
  t = clamp(t, 0.0, 1.0);

  // 3) 보간하여 현재 steer_gain 계산 (누적X, 매번 계산)
  g_steer_gain = g_steer_gain_min + t * (g_steer_gain_max - g_steer_gain_min);

  ROS_INFO_THROTTLE(0.5,
    "[lane_AB_left] center_curv=%.4e |k|=%.4e t=%.3f -> steer_gain=%.3f (range=%.3f~%.3f)",
    curv, k, t, g_steer_gain, g_steer_gain_min, g_steer_gain_max);
}

// =====================================================
// ★ main_node.cpp 에서 부를 함수들 ★
// =====================================================

void mission_AB_left_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[lane_AB_left] mission_AB_left_init()");

  // --- 파라미터 로드 ---

  // Topics
  pnh.param<std::string>("topic_left_center_point",
                         g_topic_left_center_point,
                         std::string("/perception/left_center_px"));

  pnh.param<std::string>("topic_dx_px", g_topic_dx_px,
                         std::string("/perception/dx_px")); // 참고용

  pnh.param<std::string>("motor_topic", g_motor_topic,
                         std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic", g_servo_topic,
                         std::string("/commands/servo/position"));
  pnh.param<std::string>("topic_center_color",
                         g_topic_center_color,
                         std::string("/perception/center_color_px"));

  // is_crosswalk topic
  pnh.param<std::string>("is_crosswalk_topic",
                         g_is_crosswalk_topic,
                         std::string("/perception/is_crosswalk"));

  // curvature_center topic
  pnh.param<std::string>("topic_curvature_center",
                         g_topic_curvature_center,
                         std::string("/perception/curvature_center"));

  ROS_INFO("[lane_AB_left] subscribe left_center='%s'",
           ros::names::resolve(g_topic_left_center_point).c_str());
  ROS_INFO("[lane_AB_left] subscribe center_color='%s'",
           ros::names::resolve(g_topic_center_color).c_str());
  ROS_INFO("[lane_AB_left] subscribe is_crosswalk='%s'",
           ros::names::resolve(g_is_crosswalk_topic).c_str());
  ROS_INFO("[lane_AB_left] subscribe curvature_center='%s'",
           ros::names::resolve(g_topic_curvature_center).c_str());
  ROS_INFO("[lane_AB_left] publish motor='%s', servo='%s'",
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str());

  // BEV 중심 x (왼쪽 차선 기준으로 튜닝)
  pnh.param<double>("bev_center_x_px", g_bev_center_x_px, 320.0);

  // 서보 스케일
  pnh.param<double>("servo_center", g_servo_center, 0.5);
  pnh.param<double>("servo_min",    g_servo_min,    0.0);
  pnh.param<double>("servo_max",    g_servo_max,    1.0);
  pnh.param<double>("steer_sign",   g_steer_sign,  -1.0);

  // Control params
  pnh.param<double>("max_abs_dx_px", g_max_abs_dx_px, 83.0);
  pnh.param<double>("dx_tolerance",  g_dx_tolerance,  10.0);
  pnh.param<double>("steer_gain",        g_steer_gain_base, 0.8);
  pnh.param<double>("steer_gain_min",    g_steer_gain_min,  0.4);
  pnh.param<double>("steer_gain_max",    g_steer_gain_max,  2.0);
  g_steer_gain = g_steer_gain_base;
  pnh.param<double>("steer_smoothing_alpha",       g_alpha_ema, 0.2);
  pnh.param<double>("max_steer_delta_per_cycle",   g_max_delta, 0.08);
  pnh.param<double>("follow_duration_sec", g_follow_duration_sec, 8.0);

  // 속도 계획
  pnh.param<double>("base_speed_mps",  g_base_speed_mps,  7.0);
  pnh.param<double>("min_speed_mps",   g_min_speed_mps,   0.8);
  pnh.param<double>("speed_drop_gain", g_speed_drop_gain, 0.5);

  // 모터 스케일
  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);
  pnh.param<double>("motor_max_cmd", g_motor_max_cmd, 1200.0);
  pnh.param<double>("motor_gain",    g_motor_gain,    300.0);

  // PID 파라미터
  pnh.param<double>("pid_kp",     g_pid_kp,     0.01);
  pnh.param<double>("pid_ki",     g_pid_ki,     0.0);
  pnh.param<double>("pid_kd",     g_pid_kd,     0.0001);
  pnh.param<double>("pid_weight", g_pid_weight, 0.05);

  // 타임아웃
  pnh.param<double>("dx_timeout_sec", g_dx_timeout_sec, 1.0);

  // 곡률 범위 파라미터
  pnh.param<double>("min_curvature", g_min_curv, 3e-6);
  pnh.param<double>("max_curvature", g_max_curv, 1e-3);

  // --- Pub/Sub ---
  g_left_center_sub   = nh.subscribe(g_topic_left_center_point, 20, leftCenterCB);
  g_center_color_sub  = nh.subscribe(g_topic_center_color,      10, centerColorCB);
  g_crosswalk_sub     = nh.subscribe(g_is_crosswalk_topic,      10, crosswalkCB);
  g_curvature_sub     = nh.subscribe(g_topic_curvature_center,  10, curvatureCenterCB);

  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_pub_servo = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);

  g_follow_running = false;
  g_sequence_done = false;

  ROS_INFO("[lane_AB_left] mission_AB_left_init done");
}

// main_node.cpp 의 while 루프 안에서 매 주기마다 호출
void mission_AB_left_step()
{
  ros::Time now = ros::Time::now();

  if (!g_follow_running) {
    g_follow_running = true;
    g_follow_start = now;
    g_sequence_done = false;
    ROS_INFO("[lane_AB_left] sequence start (%.1fs)", g_follow_duration_sec);
  }

  double seq_elapsed = (now - g_follow_start).toSec();
  if (seq_elapsed >= g_follow_duration_sec) {
    g_sequence_done = true;
    std_msgs::Float64 motor_msg;
    std_msgs::Float64 servo_msg;
    motor_msg.data = 0.0;
    servo_msg.data = g_servo_center;
    g_pub_motor.publish(motor_msg);
    g_pub_servo.publish(servo_msg);
    ROS_INFO_THROTTLE(1.0, "[lane_AB_left] sequence done");
    return;
  }

  bool have_dx = false;

  if (g_have_cb_time) {
    double dt = (now - g_last_cb_time).toSec();
    have_dx = (dt <= g_dx_timeout_sec);
  }

  double steer_cmd = 0.0;
  double speed_cmd = 0.0;

  if (have_dx) {
    steer_cmd = g_latest_steer_cmd;  // -1 ~ +1
    speed_cmd = g_latest_speed_cmd;  // m/s
  } else {
    steer_cmd = 0.0;
    speed_cmd = 0.0;
  }

  // ---------------- is_crosswalk 처리 (7초 정지) ----------------
  if (g_is_crosswalk && !g_crosswalk_timer_running) {
    g_crosswalk_timer_running = true;
    g_crosswalk_start_time    = now;
    ROS_INFO("[lane_AB_left] is_crosswalk TRUE: start 7s hold");
  }

  if (g_crosswalk_timer_running) {
    double dt = (now - g_crosswalk_start_time).toSec();
    if (dt < 7.0) {
      steer_cmd = 0.0;
      speed_cmd = 0.0; // STOP 

      ROS_INFO_THROTTLE(1.0,
        "[lane_AB_left] is_crosswalk: HOLD (t=%.2f/7.0)", dt);
    } else {
      g_crosswalk_timer_running = false;
      ROS_INFO("[lane_AB_left] is_crosswalk: 7s hold finished");
    }
  }
  // -------------------------------------------------------------

  // ---- 1) 조향 변환: -1~+1 -> 서보 0~1 ----
  double steer_norm = clamp(steer_cmd, -1.0, 1.0);

  // 방향 뒤집기
  steer_norm *= (-g_steer_sign);

  // 0.5 = 직진, ±servo_range 안에서 사용
  double servo_range = std::min(g_servo_center - g_servo_min,
                                g_servo_max - g_servo_center);
  double servo_hw = g_servo_center + steer_norm * servo_range;
  servo_hw = clamp(servo_hw, g_servo_min, g_servo_max);

  // ---- 2) 속도 변환: m/s -> 모터 명령 ----
  double motor_cmd = g_motor_gain * speed_cmd;
  motor_cmd = clamp(motor_cmd, g_motor_min_cmd, g_motor_max_cmd);

  // ---- 3) Publish ----
  std_msgs::Float64 motor_msg;
  std_msgs::Float64 servo_msg;
  motor_msg.data = motor_cmd;
  servo_msg.data = servo_hw;

  g_pub_motor.publish(motor_msg);
  g_pub_servo.publish(servo_msg);

  // 필요하면 디버그 로그 해제
  // ROS_INFO_THROTTLE(0.5,
  //   "[lane_AB_left][loop] have_dx=%d steer_cmd=%.3f servo=%.3f motor=%.1f v=%.2f (steer_gain=%.3f)",
  //   (int)have_dx, steer_cmd, servo_hw, motor_cmd, speed_cmd, g_steer_gain);
}

bool mission_AB_left_is_done()
{
  return g_sequence_done;
}

// -------------------------------------------------------------
// main_node.cpp에서 기대하는 snake_case 래퍼들
// -------------------------------------------------------------
void mission_ab_left_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  mission_AB_left_init(nh, pnh);
}

void mission_ab_left_step()
{
  mission_AB_left_step();
}

bool mission_ab_left_done()
{
  return mission_AB_left_is_done();
}

void mission_ab_left_reset()
{
  g_follow_running = false;
  g_sequence_done = false;
  g_follow_start = ros::Time(0);
}

