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
std::string g_topic_center_point;
std::string g_topic_dx_px;   // 참고용(실제 구독 X)
std::string g_motor_topic;
std::string g_servo_topic;
std::string g_is_crosswalk_topic;
std::string g_topic_curvature_center;   // ★ center curvature 토픽 이름
std::string g_topic_center_color; // 1124 새로추가

int g_lane_color_code = 0;  // 0 = none, 1 = red, 2 = blue // 1124 새로추가

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
double g_prev_steer        = 0.0; // 내부 steer_cmd (-1 ~ +1)
double g_latest_steer_cmd  = 0.0;
double g_latest_speed_cmd  = 0.0;
ros::Time g_last_cb_time;
bool g_have_cb_time = false;

// PID 파라미터
double g_pid_kp = 0.02;    // 1124 새로추가
double g_pid_ki = 0.0;     // 1124 새로추가
double g_pid_kd = 0.001;   // 1124 새로추가
double g_pid_weight = 0.3;  // tanh 기반 조향에 더해지는 PID 비중 // 1124 새로추가

// PID 내부 상태
double g_pid_integral = 0.0; // 1124 새로추가
double g_pid_prev_error = 0.0; // 1124 새로추가
ros::Time g_pid_prev_time; // 1124 새로추가
bool g_pid_has_prev_t = false; // 1124 새로추가

// 곡률 범위 파라미터
double g_min_curv = 3e-4;    // 최소 곡률 (예: 직선에 가까움)
double g_max_curv = 1.5e-3;  // 최대 곡률 (예: 많이 휘어짐)

// -------------------- PID 한 스텝 계산 --------------------
double runPID(double error)
{
  // 1124 새로추가: PID 계산 함수
  ros::Time now = ros::Time::now();
  double dt = 0.0;

  if (g_pid_has_prev_t) {
    dt = (now - g_pid_prev_time).toSec();
  }
  g_pid_prev_time = now;
  g_pid_has_prev_t = true;

  // P 항
  double P = g_pid_kp * error;

  // I 항
  g_pid_integral += error * dt;
  double I = g_pid_ki * g_pid_integral;

  // D 항
  double D = 0.0;
  if (dt > 1e-4) {
    double diff = (error - g_pid_prev_error) / dt;
    D = g_pid_kd * diff;
  }
  g_pid_prev_error = error;

  return P + I + D;  // px 단위 raw 출력
}

// -------------------- dx 처리 로직 --------------------
void processDx(double dx)
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
  double u_pid = runPID(dx);                 // px 단위 // 1124 새로추가
  double steer_pid = clamp(u_pid / g_max_abs_dx_px, -1.0, 1.0); // 1124 새로추가

  const double w_base = 1.0;
  double steer_raw = w_base * steer_raw_base + g_pid_weight * steer_pid; // 1124 새로추가
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

  // 색상 기반 속도 조정 // 1124 새로추가
  if (g_lane_color_code == 1) { // 1124 새로추가
    // 빨강 → 감속
    speed_cmd *= 0.7;
  } else if (g_lane_color_code == 2) { // 1124 새로추가
    // 파랑 → 속도 복원
    speed_cmd = g_base_speed_mps;
  }

  g_latest_steer_cmd = steer_cmd;   // -1 ~ +1
  g_latest_speed_cmd = speed_cmd;   // m/s 개념

  ROS_INFO_THROTTLE(0.5,
    "[lane_ctrl][CB+PID] dx=%.1fpx errN=%.3f steer=%.3f v=%.2f (steer_gain=%.3f, Kp=%.4f, Ki=%.4f, Kd=%.4f, color=%d)",
    dx, err_norm, steer_cmd, speed_cmd, g_steer_gain, g_pid_kp, g_pid_ki, g_pid_kd, g_lane_color_code);
}

// -------------------- 콜백: center_point_px --------------------
void centerCB(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  // 타임아웃 체크용 시간 저장
  g_last_cb_time = ros::Time::now();
  g_have_cb_time = true;

  double cx = msg->point.x;
  double dx = cx - g_bev_center_x_px;   // 오른쪽이 +, 왼쪽이 -

  processDx(dx);
}

// -------------------- 콜백: center_color --------------------
void centerColorCB(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  g_lane_color_code = static_cast<int>(msg->point.x); // 0,1,2(red/blue) // 1124 새로추가
}

// -------------------- 콜백: curvature_center --------------------
void curvatureCenterCB(const std_msgs::Float32::ConstPtr& msg)
{
  double center_curv = msg->data;

  double denom = (g_max_curv - g_min_curv);
  if (denom == 0.0) {
    ROS_WARN_THROTTLE(1.0, "[lane_ctrl] (max_curv - min_curv) == 0, skip curvature scaling");
    return;
  }

  // 1) 중심곡률 > 최대곡률
  if (center_curv > g_max_curv) {
    double ratio = (center_curv - g_max_curv) / denom;
    g_steer_gain = g_steer_gain * 1.0 + ratio;
  }
  // 2) 중심곡률 < 최소곡률
  else if (center_curv < g_min_curv) {
    double ratio = (g_max_curv - center_curv) / denom;
    g_steer_gain = g_steer_gain * ratio;
  }
  // 3) 임계 사이 (min_curv <= center_curv <= max_curv)
  else {
    double ratio = (g_max_curv - center_curv) / denom;
    g_steer_gain = g_steer_gain * ratio;
  }

  ROS_INFO_THROTTLE(0.5,
    "[lane_ctrl] center_curv=%.4e (min=%.4e, max=%.4e) -> steer_gain=%.3f",
    center_curv, g_min_curv, g_max_curv, g_steer_gain);
}


// -------------------- main (단독 차선 노드) --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_center_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("lane_center_controller (center_point_px -> motor/servo Float64)");

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

  // is_crosswalk topic
  pnh.param<std::string>("is_crosswalk_topic",
                         g_is_crosswalk_topic,
                         std::string("/perception/is_crosswalk"));

  // curvature_center topic
  pnh.param<std::string>("topic_curvature_center",
                         g_topic_curvature_center,
                         std::string("/perception/curvature_center"));

  ROS_INFO("[lane_ctrl] subscribe center='%s'",
           ros::names::resolve(g_topic_center_point).c_str());
  ROS_INFO("[lane_ctrl] subscribe center_color='%s'",
           ros::names::resolve(g_topic_center_color).c_str()); // 1124 새로추가
  ROS_INFO("[lane_ctrl] subscribe is_crosswalk='%s'",
           ros::names::resolve(g_is_crosswalk_topic).c_str());
  ROS_INFO("[lane_ctrl] subscribe curvature_center='%s'",
           ros::names::resolve(g_topic_curvature_center).c_str());
  ROS_INFO("[lane_ctrl] publish motor='%s', servo='%s'",
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str());

  // BEV 중심 x
  pnh.param<double>("bev_center_x_px", g_bev_center_x_px, 320.0);

  // 서보 스케일
  pnh.param<double>("servo_center", g_servo_center, 0.5);
  pnh.param<double>("servo_min",    g_servo_min,    0.0);
  pnh.param<double>("servo_max",    g_servo_max,    1.0);
  pnh.param<double>("steer_sign",   g_steer_sign,  -1.0);

  // Control params
  pnh.param<double>("max_abs_dx_px", g_max_abs_dx_px, 83.0); // 83,
  pnh.param<double>("dx_tolerance",  g_dx_tolerance,  3.0);
  pnh.param<double>("steer_gain",    g_steer_gain,    1.5); // 초기 steer_gain
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

  // PID 파라미터
  pnh.param<double>("pid_kp", g_pid_kp, 0.02); // 1124 새로추가
  pnh.param<double>("pid_ki", g_pid_ki, 0.0);  // 1124 새로추가
  pnh.param<double>("pid_kd", g_pid_kd, 0.001); // 1124 새로추가
  pnh.param<double>("pid_weight", g_pid_weight, 0.3); // 1124 새로추가

  // 타임아웃
  pnh.param<double>("dx_timeout_sec", g_dx_timeout_sec, 1.0);

  // 곡률 범위 파라미터
  pnh.param<double>("min_curvature", g_min_curv, 0.0);
  pnh.param<double>("max_curvature", g_max_curv, 0.01);

  // --- Pub/Sub ---
  ros::Subscriber center_sub     = nh.subscribe(g_topic_center_point,       20, centerCB);
  ros::Subscriber center_color_sub = nh.subscribe(g_topic_center_color,     10, centerColorCB); // 1124 새로추가
  ros::Subscriber curvature_sub  = nh.subscribe(g_topic_curvature_center,   10, curvatureCenterCB);

  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_pub_servo = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);

  // --- 메인 루프 ---
  ros::Rate rate(15);
  while (ros::ok()) {
    ros::spinOnce();

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
      ROS_INFO_THROTTLE(1.0,
        "[lane_ctrl] waiting /perception/center_point_px ... (timeout)");
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

    ROS_INFO_THROTTLE(0.5,
      "[lane_ctrl][loop] have_dx=%d steer_cmd=%.3f servo=%.3f motor=%.1f v=%.2f (steer_gain=%.3f)",
      (int)have_dx, steer_cmd, servo_hw, motor_cmd, speed_cmd, g_steer_gain);

    rate.sleep();
  }

  return 0;
}
