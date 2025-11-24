/******************************************************
 * mission_ab_toMain.cpp
 ******************************************************/
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>

#include <cmath>
#include <string>

// -------------------- 유틸 --------------------
inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- 전역 상태 --------------------
// 토픽 이름들
static std::string g_topic_center_point;
static std::string g_topic_dx_px;              // 참고용(실제 구독 X)
static std::string g_motor_topic;
static std::string g_servo_topic;

// BEV 중심 x 픽셀
static double g_bev_center_x_px = 320.0;

// 서보 스케일
static double g_servo_center = 0.5;
static double g_servo_min    = 0.0;
static double g_servo_max    = 1.0;
static double g_steer_sign   = -1.0;

// Control 파라미터 (조향/속도)
static double g_max_abs_dx_px = 83.0;
static double g_dx_tolerance  = 3.0;
static double g_alpha_ema     = 0.2;
static double g_max_delta     = 0.08;

// 속도 계획 (dx 기반)
static double g_base_speed_mps  = 7.0;
static double g_min_speed_mps   = 0.8;
static double g_speed_drop_gain = 0.5;

// 모터 스케일
static double g_motor_min_cmd = 0.0;   // ← 여기 param으로 1000 이상으로 올려도 됨
static double g_motor_max_cmd = 900.0;
static double g_motor_gain    = 300.0;

// 타임아웃
static double g_dx_timeout_sec = 1.0;

// 퍼블리셔
static ros::Publisher g_pub_motor;
static ros::Publisher g_pub_servo;

// 서브스크라이버
static ros::Subscriber g_center_sub;

// 내부 상태 (조향 출력을 위한)
static double    g_prev_steer        = 0.0;
static double    g_latest_steer_cmd  = 0.0;
static double    g_latest_speed_cmd  = 0.0;
static ros::Time g_last_cb_time;
static bool      g_have_cb_time = false;

// -------------------- PID (dx -> steer) --------------------
static double g_kp = 1.5;   // 비례 게인
static double g_ki = 0.0;   // 적분 게인
static double g_kd = 0.0;   // 미분 게인

static double    g_err_int  = 0.0;
static double    g_prev_err = 0.0;
static bool      g_have_prev_err_for_pid = false;
static double    g_int_max = 1.0;  // 적분 한계

static ros::Time g_prev_pid_time;  // dt 계산 용

// -------------------- dx 처리 로직 (PID + 속도 계획) --------------------
static void processDx(double dx)
{
  // 1) 에러 정규화
  double err_norm = 0.0;
  if (std::fabs(dx) <= g_dx_tolerance) {
    err_norm = 0.0;
  } else {
    err_norm = clamp(dx / g_max_abs_dx_px, -1.0, 1.0);
  }

  // 2) 시간 계산 (PID dt)
  ros::Time now = ros::Time::now();
  double dt = 0.0;
  if (!g_prev_pid_time.isZero()) {
    dt = (now - g_prev_pid_time).toSec();
  }
  g_prev_pid_time = now;

  double e = err_norm;

  // 3) PID 계산
  double u = 0.0;
  if (dt > 0.0) {
    // 적분
    g_err_int += e * dt;
    g_err_int = clamp(g_err_int, -g_int_max, g_int_max);

    // 미분
    double de = 0.0;
    if (g_have_prev_err_for_pid) {
      de = (e - g_prev_err) / dt;
    }
    g_prev_err = e;
    g_have_prev_err_for_pid = true;

    u = g_kp * e + g_ki * g_err_int + g_kd * de;
  } else {
    // 첫 프레임 등 dt==0일 때는 P만 사용
    u = g_kp * e;
  }

  // -1 ~ +1 범위로 제한
  double steer_raw = clamp(u, -1.0, 1.0);

  // 4) EMA 스무딩
  double steer_smooth = g_alpha_ema * steer_raw + (1.0 - g_alpha_ema) * g_prev_steer;

  // 5) 한 step당 변화량 제한
  double delta = clamp(steer_smooth - g_prev_steer, -g_max_delta, g_max_delta);
  double steer_cmd = g_prev_steer + delta;
  g_prev_steer = steer_cmd;

  // ---------------- 속도 계획 (dx 기반만 사용) ----------------
  // 에러 클수록 감속
  double speed_cmd = g_base_speed_mps - g_speed_drop_gain * std::fabs(err_norm);
  speed_cmd = clamp(speed_cmd, g_min_speed_mps, g_base_speed_mps);

  g_latest_steer_cmd = steer_cmd;   // -1 ~ +1
  g_latest_speed_cmd = speed_cmd;   // m/s 개념

  ROS_INFO_THROTTLE(0.5,
    "[lane_ctrl][CB] dx=%.1fpx e=%.3f I=%.3f steer=%.3f v=%.2f (Kp=%.2f Ki=%.2f Kd=%.2f)",
    dx, e, g_err_int, steer_cmd, speed_cmd, g_kp, g_ki, g_kd);
}

// -------------------- 콜백: center_point_px --------------------
static void centerCB(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  // 타임아웃 체크용 시간 저장
  g_last_cb_time = ros::Time::now();
  g_have_cb_time = true;

  double cx = msg->point.x;
  double dx = cx - g_bev_center_x_px;   // 오른쪽이 +, 왼쪽이 -

  processDx(dx);
}


// =====================================================
// main_node.cpp 에서 부를 함수들
// =====================================================

void mission_lane_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[lane_ctrl] mission_lane_init()");

  // --- 토픽 파라미터 로드 ---
  pnh.param<std::string>("topic_center_point",
                         g_topic_center_point,
                         std::string("/perception/center_point_px"));
  pnh.param<std::string>("topic_dx_px", g_topic_dx_px,
                         std::string("/perception/dx_px")); // 참고용

  pnh.param<std::string>("motor_topic", g_motor_topic,
                         std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic", g_servo_topic,
                         std::string("/commands/servo/position"));

  ROS_INFO("[lane_ctrl] subscribe center='%s'",
           ros::names::resolve(g_topic_center_point).c_str());
  ROS_INFO("[lane_ctrl] publish motor='%s', servo='%s'",
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str());

  // --- 기본 파라미터 로드 ---

  // BEV 중심 x
  pnh.param<double>("bev_center_x_px", g_bev_center_x_px, 320.0);

  // 서보 스케일
  pnh.param<double>("servo_center", g_servo_center, 0.5);
  pnh.param<double>("servo_min",    g_servo_min,    0.0);
  pnh.param<double>("servo_max",    g_servo_max,    1.0);
  pnh.param<double>("steer_sign",   g_steer_sign,  -1.0);

  // Control params (dx 관련)
  pnh.param<double>("max_abs_dx_px", g_max_abs_dx_px, 83.0);
  pnh.param<double>("dx_tolerance",  g_dx_tolerance,  3.0);
  pnh.param<double>("steer_smoothing_alpha", g_alpha_ema, 0.2);
  pnh.param<double>("max_steer_delta_per_cycle", g_max_delta, 0.08);

  // 속도 계획
  pnh.param<double>("base_speed_mps",  g_base_speed_mps,  7.0);
  pnh.param<double>("min_speed_mps",   g_min_speed_mps,   0.8);
  pnh.param<double>("speed_drop_gain", g_speed_drop_gain, 0.5);

  // 모터 스케일
  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);
  pnh.param<double>("motor_max_cmd", g_motor_max_cmd, 1000.0);
  pnh.param<double>("motor_gain",    g_motor_gain,    300.0);

  // 타임아웃
  pnh.param<double>("dx_timeout_sec", g_dx_timeout_sec, 1.0);

  // PID 파라미터
  double steer_gain_legacy = 1.5;
  pnh.param<double>("steer_gain", steer_gain_legacy, 1.5);
  g_kp = steer_gain_legacy;

  pnh.param<double>("kp_lat", g_kp, g_kp);
  pnh.param<double>("ki_lat", g_ki, 0.0);
  pnh.param<double>("kd_lat", g_kd, 0.0);
  pnh.param<double>("int_max_lat", g_int_max, 1.0);

  // --- Pub/Sub ---
  g_center_sub = nh.subscribe(g_topic_center_point, 20, centerCB);

  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_pub_servo = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);

  // 초기 PID 타임/상태
  g_prev_pid_time = ros::Time(0);
  g_err_int = 0.0;
  g_prev_err = 0.0;
  g_have_prev_err_for_pid = false;

  ROS_INFO("[lane_ctrl] mission_lane_init done (Kp=%.2f, Ki=%.2f, Kd=%.2f)",
           g_kp, g_ki, g_kd);
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
    // 입력 끊기면 안전하게 정지
    steer_cmd = 0.0;
    speed_cmd = 0.0;
  }

  // ---- 1) 조향 변환: -1~+1 -> 서보 0~1 ----
  double steer_norm = clamp(steer_cmd, -1.0, 1.0);

  // 방향 뒤집기
  steer_norm *= (-g_steer_sign);

  // 0.5 = 직진, ±servo_range 안에서 사용
  double servo_range = std::min(g_servo_center - g_servo_min,
                                g_servo_max   - g_servo_center);
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
    "[lane_ctrl][loop] have_dx=%d steer_cmd=%.3f servo=%.3f motor=%.1f v=%.2f",
    (int)have_dx, steer_cmd, servo_hw, motor_cmd, speed_cmd);
}

// -------------------- main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_ab_to_main");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  mission_lane_init(nh, pnh);

  ros::Rate rate(30.0);
  while (ros::ok())
  {
    ros::spinOnce();
    mission_lane_step();
    rate.sleep();
  }
  return 0;
}
