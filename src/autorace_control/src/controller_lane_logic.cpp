#include "controller_lane_logic.hpp"

// 여기서는 extern 전역들을 그대로 사용한다.
// 실제 정의는 controller_lane_wego.cpp 안에 있음.

// -------------------- 파라미터 로딩 --------------------
void loadParams(ros::NodeHandle& pnh)
{
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

  // is_crosswalk topic
  pnh.param<std::string>("is_crosswalk_topic",
                         g_is_crosswalk_topic,
                         std::string("/perception/is_crosswalk"));

  // curvature_center topic
  pnh.param<std::string>("topic_curvature_center",
                         g_topic_curvature_center,
                         std::string("/perception/curvature_center"));

  // BEV 중심 x
  pnh.param<double>("bev_center_x_px", g_bev_center_x_px, 320.0);

  // 서보 스케일
  pnh.param<double>("servo_center", g_servo_center, 0.5);
  pnh.param<double>("servo_min",    g_servo_min,    0.0);
  pnh.param<double>("servo_max",    g_servo_max,    1.0);
  pnh.param<double>("steer_sign",   g_steer_sign,  -1.0);

  // Control params
  pnh.param<double>("max_abs_dx_px", g_max_abs_dx_px, 83.0);
  pnh.param<double>("dx_tolerance",  g_dx_tolerance,  3.0);
  pnh.param<double>("steer_gain",    g_steer_gain,    1.5);
  pnh.param<double>("steer_smoothing_alpha", g_alpha_ema, 0.2);
  pnh.param<double>("max_steer_delta_per_cycle", g_max_delta, 0.08);

  // 속도 계획
  pnh.param<double>("base_speed_mps",  g_base_speed_mps,  7.0);
  pnh.param<double>("min_speed_mps",   g_min_speed_mps,   0.8);
  pnh.param<double>("speed_drop_gain", g_speed_drop_gain, 0.5);

  // 모터 스케일
  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);
  pnh.param<double>("motor_max_cmd", g_motor_max_cmd, 900.0);
  pnh.param<double>("motor_gain",    g_motor_gain,    300.0);

  // 타임아웃
  pnh.param<double>("dx_timeout_sec", g_dx_timeout_sec, 1.0);

  // 곡률 범위 파라미터
  pnh.param<double>("min_curvature", g_min_curv, 0.0003);
  pnh.param<double>("max_curvature", g_max_curv, 0.0015);
}

// -------------------- dx 처리 로직 --------------------
void compute_Dx(double dx)
{
  double err_norm = 0.0;

  if (std::fabs(dx) <= g_dx_tolerance) {
    err_norm = 0.0;
  } else {
    err_norm = clamp(dx / g_max_abs_dx_px, -1.0, 1.0);
  }

  // -1 ~ +1 사이 조향 명령 (tanh 비선형)
  double steer_raw = std::tanh(g_steer_gain * err_norm);

  // EMA 스무딩
  double steer_smooth = g_alpha_ema * steer_raw + (1.0 - g_alpha_ema) * g_prev_steer;

  // 한 step당 변화 제한
  double delta = clamp(steer_smooth - g_prev_steer, -g_max_delta, g_max_delta);
  double steer_cmd = g_prev_steer + delta;
  g_prev_steer = steer_cmd;

  // 속도: 오차 클수록 줄이기
  double speed_cmd = clamp(g_base_speed_mps - g_speed_drop_gain * std::fabs(err_norm),
                           g_min_speed_mps, g_base_speed_mps);

  g_latest_steer_cmd = steer_cmd;   // -1 ~ +1
  g_latest_speed_cmd = speed_cmd;   // m/s 개념

  ROS_INFO_THROTTLE(0.5,
    "[lane_ctrl][CB] dx=%.1fpx errN=%.3f steer=%.3f v=%.2f (steer_gain=%.3f)",
    dx, err_norm, steer_cmd, speed_cmd, g_steer_gain);
}

// -------------------- 콜백: center_point_px --------------------
void CB_center(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  g_last_cb_time = ros::Time::now();
  g_have_cb_time = true;

  double cx = msg->point.x;
  double dx = cx - g_bev_center_x_px;   // 오른쪽이 +, 왼쪽이 -

  compute_Dx(dx);
}

// -------------------- 콜백: is_crosswalk --------------------
void CB_crosswalk(const std_msgs::Bool::ConstPtr& msg)
{
  g_is_crosswalk = msg->data;
}

// -------------------- 콜백: curvature_center --------------------
void CB_center_curvature(const std_msgs::Float32::ConstPtr& msg)
{
  double center_curv = msg->data;

  double denom = (g_max_curv - g_min_curv);
  if (denom == 0.0) {
    ROS_WARN_THROTTLE(1.0, "[lane_ctrl] (max_curv - min_curv) == 0, skip curvature scaling");
    return;
  }

  if (center_curv > g_max_curv) {
    double ratio = (center_curv - g_max_curv) / denom;
    g_steer_gain = g_steer_gain * 1.0 + ratio;
  }
  else if (center_curv < g_min_curv) {
    double ratio = (g_max_curv - center_curv) / denom;
    g_steer_gain = g_steer_gain * ratio;
  }
  else {
    double ratio = (g_max_curv - center_curv) / denom;
    g_steer_gain = g_steer_gain * ratio;
  }

  ROS_INFO_THROTTLE(0.5,
    "[lane_ctrl] center_curv=%.4e (min=%.4e, max=%.4e) -> steer_gain=%.3f",
    center_curv, g_min_curv, g_max_curv, g_steer_gain);
}


// -------------------- crosswalk 7초 정지 보정 --------------------
void apply_crosswalk_hold(double& steer_cmd,
                          double& speed_cmd,
                          const ros::Time& now)
{
  // crosswalk 토픽이 true로 바뀌는 순간 타이머 시작
  if (g_is_crosswalk && !g_crosswalk_timer_running) {
    g_crosswalk_timer_running = true;
    g_crosswalk_start_time    = now;
    ROS_INFO("[lane_ctrl] is_crosswalk TRUE: start 7s hold");
  }

  // 타이머가 안 돌고 있으면 아무 것도 안 함
  if (!g_crosswalk_timer_running) {
    return;
  }

  // 타이머가 도는 중이면 남은 시간에 따라 보정
  double dt = (now - g_crosswalk_start_time).toSec();
  if (dt < 7.0) {
    // 7초 동안은 무조건 정지
    steer_cmd = 0.0;
    speed_cmd = 0.0;

    ROS_INFO_THROTTLE(1.0,
      "[lane_ctrl] is_crosswalk: HOLD (t=%.2f/7.0)", dt);
  } else {
    // 7초 지나면 타이머 종료
    g_crosswalk_timer_running = false;
    ROS_INFO("[lane_ctrl] is_crosswalk: 7s hold finished");
  }
}



// -------------------- HW 명령 변환 + Publish --------------------
void publish_motor_commands(double steer_cmd, double speed_cmd)
{
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

  ROS_INFO_THROTTLE(0.5,
    "[lane_ctrl][pub] steer_cmd=%.3f servo=%.3f motor=%.1f v=%.2f",
    steer_cmd, servo_hw, motor_cmd, speed_cmd);
}