// mission_lane.cpp
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <cmath>
#include <string>

// -------------------- 유틸 --------------------
inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- 전역 상태 --------------------
// [MOD-static] : 이 파일 안에서만 쓰는 전역은 static 붙여서
//               다른 mission_*.cpp 와 이름이 겹쳐도 링크 에러 안 나게 함.

// 토픽 이름들
static std::string g_topic_center_point;        // [MOD-static]
static std::string g_topic_dx_px;              // 참고용(실제 구독 X) [MOD-static]
static std::string g_motor_topic;              // [MOD-static]
static std::string g_servo_topic;              // [MOD-static]
static std::string g_topic_curvature_center;   // [MOD-static]
static std::string g_topic_center_color;       // 1124 새로추가 [MOD-static]

// BEV 중심 x 픽셀
static double g_bev_center_x_px = 320.0;       // [MOD-static]

// 서보 스케일
static double g_servo_center = 0.5;            // [MOD-static]
static double g_servo_min    = 0.0;            // [MOD-static]
static double g_servo_max    = 1.0;            // [MOD-static]
static double g_steer_sign   = -1.0;           // [MOD-static]

// Control 파라미터
static double g_max_abs_dx_px   = 83.0;          // [MOD-static]
static double g_dx_tolerance    = 3.0;           // [MOD-static]
static double g_steer_gain_base = 1.5;           // 곡률 반영 전 기본 gain [MOD-static]
static double g_steer_gain_min  = 0.8;           // 직선 부근에서 최소 gain [MOD-static]
static double g_steer_gain_max  = 2.0;           // 급커브에서 최대 gain [MOD-static]
static double g_steer_gain      = 1.5;           // 실제 사용 gain (매 프레임 갱신) [MOD-static]
static double g_alpha_ema       = 0.2;           // [MOD-static]
static double g_max_delta       = 0.08;          // [MOD-static]

// 속도 계획
static double g_base_speed_mps  = 7.0;         // [MOD-static]
static double g_min_speed_mps   = 0.8;         // [MOD-static]
static double g_speed_drop_gain = 0.5;         // [MOD-static]

// 모터 스케일
static double g_motor_min_cmd = 0.0;           // [MOD-static]
static double g_motor_max_cmd = 1200.0;        // 기본 상한 [MOD-static]
static double g_motor_max_cmd_red = 900.0;     // 색상(red) 감속용 [MOD-static]
static double g_motor_max_cmd_blue = 2000.0;   // 색상(blue) 가속용 [MOD-static]
static double g_motor_gain    = 300.0;         // [MOD-static]

// 타임아웃
static double g_dx_timeout_sec = 1.0;          // [MOD-static]

// 퍼블리셔
static ros::Publisher g_pub_motor;             // [MOD-static]
static ros::Publisher g_pub_servo;             // [MOD-static]

// 서브스크라이버 (init 안에서 초기화)
static ros::Subscriber g_center_sub;           // [MOD-static]
static ros::Subscriber g_center_color_sub;     // 1124 새로추가 [MOD-static]
static ros::Subscriber g_curvature_sub;        // [MOD-static]

// 내부 상태
static double g_prev_steer        = 0.0;       // [MOD-static]
static double g_latest_steer_cmd  = 0.0;       // [MOD-static]
static double g_latest_speed_cmd  = 0.0;       // [MOD-static]
static ros::Time g_last_cb_time;               // [MOD-static]
static bool g_have_cb_time = false;            // [MOD-static]

// 곡률 범위 파라미터
static double g_min_curv = 3e-6;              // [MOD-static]
static double g_max_curv = 1e-3;              // [MOD-static]

// PID 파라미터
static double g_pid_kp = 0.02;               // 1124 새로추가 [MOD-static]
static double g_pid_ki = 0.0;                // 1124 새로추가 [MOD-static]
static double g_pid_kd = 0.001;              // 1124 새로추가 [MOD-static]
static double g_pid_weight = 0.3;            // 1124 새로추가 [MOD-static]

// PID 내부 상태
static double g_pid_integral = 0.0;          // 1124 새로추가 [MOD-static]
static double g_pid_prev_error = 0.0;        // 1124 새로추가 [MOD-static]
static ros::Time g_pid_prev_time;            // 1124 새로추가 [MOD-static]
static bool g_pid_has_prev_t = false;        // 1124 새로추가 [MOD-static]

// 색상 정보
static int g_lane_color_code = 0;            // 0 none, 1 red, 2 blue // 1124 새로추가 [MOD-static]

// dx 스파이크 필터 // 1124 새로추가
static double g_prev_dx = 0.0;               // [MOD-static]
static bool   g_has_prev_dx = false;         // [MOD-static]

// -------------------- PID 한 스텝 계산 --------------------
static double runPID(double error)            // 1124 새로추가 [MOD-static]
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
static void processDx(double dx)               // [MOD-static] 이 파일 안에서만 쓰는 함수
{
  double abs_dx = std::fabs(dx);
  double err_norm = 0.0;

  if (abs_dx <= g_dx_tolerance) {
    err_norm = 0.0;
  } else {
    err_norm = clamp(dx / g_max_abs_dx_px, -1.0, 1.0);
  }

  // === 1) 비선형 gain: 직선(작은 dx)에서는 약하게, 큰 오차에서 더 세게 ===
  double gain_scale = 1.0;
  if (abs_dx < 15.0) {
    gain_scale = 0.5;   // 중앙 근처 흔들림 억제
  } else if (abs_dx > 35.0) {
    gain_scale = 1.35;   // 큰 오차/곡선에서 더 공격적으로
  }
  double steer_raw_base = std::tanh(g_steer_gain * gain_scale * err_norm);

  // PID 보정 (dx를 직접 에러로 사용)
  double u_pid = runPID(dx);                 // px 단위 // 1124 새로추가
  double steer_pid = clamp(u_pid / g_max_abs_dx_px, -1.0, 1.0); // 1124 새로추가

  const double w_base = 1.0;
  double steer_raw = w_base * steer_raw_base + g_pid_weight * steer_pid; // 1124 새로추가
  steer_raw = clamp(steer_raw, -1.0, 1.0);

  // === 2) EMA + delta도 |dx|에 따라 다르게 ===
  double local_alpha = g_alpha_ema;
  double local_delta = g_max_delta;

  if (abs_dx < 15.0) {
    local_alpha = 0.15;   // 중앙 근처: 더 부드럽게
    local_delta = 0.05;
  } else if (abs_dx > 35.0) {
    local_alpha = 0.28;   // 큰 오차/곡선: 더 빠르게
    local_delta = 0.12;
  }

  double steer_smooth = local_alpha * steer_raw + (1.0 - local_alpha) * g_prev_steer;

  // 한 step당 변화 제한
  double delta = clamp(steer_smooth - g_prev_steer, -local_delta, local_delta);
  double steer_cmd = g_prev_steer + delta;
  g_prev_steer = steer_cmd;

  // 속도: 오차 클수록 줄이기
  double speed_cmd = clamp(g_base_speed_mps - g_speed_drop_gain * std::fabs(err_norm),
                           g_min_speed_mps, g_base_speed_mps);

  // // 색상 기반 속도 조정 // 1124 새로추가
  // if (g_lane_color_code == 1) { // 1124 새로추가
  //   speed_cmd *= 0.5;
  // } else if (g_lane_color_code == 2) { // 1124 새로추가
  //   speed_cmd = g_base_speed_mps;
  // }

  g_latest_steer_cmd = steer_cmd;   // -1 ~ +1
  g_latest_speed_cmd = speed_cmd;   // m/s 개념

  ROS_INFO_THROTTLE(0.5,
    "[lane_ctrl][CB+PID] dx=%.1fpx |dx|=%.1f errN=%.3f steer=%.3f v=%.2f "
    "(gain_scale=%.2f, alpha=%.2f, dmax=%.2f, steer_gain=%.3f)",
    dx, abs_dx, err_norm, steer_cmd, speed_cmd,
    gain_scale, local_alpha, local_delta, g_steer_gain);
}

// -------------------- 콜백: center_point_px --------------------
static void centerCB(const geometry_msgs::PointStamped::ConstPtr& msg) // [MOD-static]
{
  // 타임아웃 체크용 시간 저장
  g_last_cb_time = ros::Time::now();
  g_have_cb_time = true;

  double cx = msg->point.x;
  double dx_raw = cx - g_bev_center_x_px;   // 오른쪽이 +, 왼쪽이 -

  // 급점프 프레임 차단 // 1124 새로추가
  double dx = dx_raw;
  const double spike_thresh = 80.0; // px 기준
  if (g_has_prev_dx && std::fabs(dx_raw - g_prev_dx) > spike_thresh) {
    dx = g_prev_dx;   // 급점프 프레임 완전 무시
  }
  g_prev_dx = dx;
  g_has_prev_dx = true;

  processDx(dx);
}

// -------------------- 콜백: center_color --------------------
static void centerColorCB(const geometry_msgs::PointStamped::ConstPtr& msg) // 1124 새로추가 [MOD-static]
{
  g_lane_color_code = static_cast<int>(msg->point.x); // 0,1,2(red/blue)
}

// -------------------- 콜백: curvature_center --------------------
static void curvatureCenterCB(const std_msgs::Float32::ConstPtr& msg) // [MOD-static]
{
  double curv = msg->data;          // 부호는 좌/우 구분, 여기서는 크기만 사용
  double k = std::fabs(curv);

  if (g_max_curv <= g_min_curv) {
    ROS_WARN_THROTTLE(1.0,
      "[lane_ctrl] invalid curvature range (min >= max), use base gain");
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
    "[lane_ctrl] center_curv=%.4e |k|=%.4e t=%.3f -> steer_gain=%.3f (range=%.3f~%.3f)",
    curv, k, t, g_steer_gain, g_steer_gain_min, g_steer_gain_max);
}


// =====================================================
// ★ main_node.cpp 에서 부를 함수들 ★
// =====================================================

void mission_lane_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[lane_ctrl] mission_lane_init()");

  // --- 파라미터 로드 ---

  // Topics
  pnh.param<std::string>("topic_center_point",
                         g_topic_center_point,
                         std::string("/perception/center_point_px"));
  pnh.param<std::string>("topic_dx_px", g_topic_dx_px,
                         std::string("/perception/dx_px")); // 참고용

  pnh.param<std::string>("motor_topic", g_motor_topic,
                         std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic", g_servo_topic,
                         std::string("/commands/servo/position"));
  pnh.param<std::string>("topic_center_color",
                         g_topic_center_color,
                         std::string("/perception/center_color_px")); // 1124 새로추가

  // curvature_center topic
  pnh.param<std::string>("topic_curvature_center",
                         g_topic_curvature_center,
                         std::string("/perception/curvature_center"));

  ROS_INFO("[lane_ctrl] subscribe center='%s'",
           ros::names::resolve(g_topic_center_point).c_str());
  ROS_INFO("[lane_ctrl] subscribe center_color='%s'",
           ros::names::resolve(g_topic_center_color).c_str()); // 1124 새로추가
  ROS_INFO("[lane_ctrl] subscribe curvature_center='%s'",
           ros::names::resolve(g_topic_curvature_center).c_str());
  ROS_INFO("[lane_ctrl] publish motor='%s', servo='%s'",
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str());

  // BEV 중심 x
  pnh.param<double>("bev_center_x_px", g_bev_center_x_px, 320.0);

  // 서보 스케일
  pnh.param<double>("servo_center", g_servo_center, 0.57);
  pnh.param<double>("servo_min",    g_servo_min,    0.0);
  pnh.param<double>("servo_max",    g_servo_max,    1.0);
  pnh.param<double>("steer_sign",   g_steer_sign,  -1.0);

  // Control params
  pnh.param<double>("max_abs_dx_px", g_max_abs_dx_px, 60.0); // 83,
  pnh.param<double>("dx_tolerance",  g_dx_tolerance,  8.0);
  pnh.param<double>("steer_gain",        g_steer_gain_base, 0.8);  // 기본 gain
  pnh.param<double>("steer_gain_min",    g_steer_gain_min,  0.6);  // 곡률 최소 구간 gain
  pnh.param<double>("steer_gain_max",    g_steer_gain_max,  2.2);  // 곡률 최대 구간 gain
  g_steer_gain = g_steer_gain_base; // 초기값
  pnh.param<double>("steer_smoothing_alpha", g_alpha_ema, 0.2);
  pnh.param<double>("max_steer_delta_per_cycle", g_max_delta, 0.08);

  // 속도 계획
  pnh.param<double>("base_speed_mps",  g_base_speed_mps,  7.0);
  pnh.param<double>("min_speed_mps",   g_min_speed_mps,   0.8);
  pnh.param<double>("speed_drop_gain", g_speed_drop_gain, 0.5);

  // 모터 스케일
  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);
  pnh.param<double>("motor_max_cmd",      g_motor_max_cmd,      1200.0);
  pnh.param<double>("motor_max_cmd_red",  g_motor_max_cmd_red,  900.0);   // red 감속 기본값
  pnh.param<double>("motor_max_cmd_blue", g_motor_max_cmd_blue, 2000.0);  // blue 가속 기본값
  pnh.param<double>("motor_gain",    g_motor_gain,    300.0);

  // PID 파라미터
  pnh.param<double>("pid_kp", g_pid_kp, 0.01); // 1124 새로추가
  pnh.param<double>("pid_ki", g_pid_ki, 0.0);  // 1124 새로추가
  pnh.param<double>("pid_kd", g_pid_kd, 0.0001); // 1124 새로추가
  pnh.param<double>("pid_weight", g_pid_weight, 0.00); // 1124 새로추가

  // 타임아웃
  pnh.param<double>("dx_timeout_sec", g_dx_timeout_sec, 1.0);

  // 곡률 범위 파라미터
  pnh.param<double>("min_curvature", g_min_curv, 5e-6);
  pnh.param<double>("max_curvature", g_max_curv, 5e-4);

  // --- Pub/Sub ---
  g_center_sub    = nh.subscribe(g_topic_center_point,     20, centerCB);
  g_center_color_sub = nh.subscribe(g_topic_center_color,  10, centerColorCB); // 1124 새로추가
  g_curvature_sub = nh.subscribe(g_topic_curvature_center, 10, curvatureCenterCB);

  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_pub_servo = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);

  ROS_INFO("[lane_ctrl] mission_lane_init done");
}

// main_node.cpp 의 while 루프 안에서 매 주기마다 호출
void mission_lane_step()
{
  ros::Time now = ros::Time::now();
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
    // ROS_INFO_THROTTLE(1.0,
    //   "[lane_ctrl] waiting /perception/center_point_px ... (timeout)");
  }

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

  // 색상별 강제 명령: 무색은 기존 스케일/클램프, red/blue는 고정 값
  if (g_lane_color_code == 1) {         // red
    motor_cmd = 900.0;
  } else if (g_lane_color_code == 2) {  // blue
    motor_cmd = 2000.0;
  } else {
    motor_cmd = clamp(motor_cmd, g_motor_min_cmd, g_motor_max_cmd);
  }

  // ---- 3) Publish ----
  std_msgs::Float64 motor_msg;
  std_msgs::Float64 servo_msg;
  motor_msg.data = motor_cmd;
  servo_msg.data = servo_hw;

  g_pub_motor.publish(motor_msg);
  g_pub_servo.publish(servo_msg);

  // ROS_INFO_THROTTLE(0.5,
  //   "[lane_ctrl][loop] have_dx=%d steer_cmd=%.3f servo=%.3f motor=%.1f v=%.2f (steer_gain=%.3f)",
  //   (int)have_dx, steer_cmd, servo_hw, motor_cmd, speed_cmd, g_steer_gain);
}
