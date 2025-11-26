// mission_ab_right.cpp
//  - AB 구간: 오른쪽 차선만 추종 (우측 오프셋) 후 8초 경과 시 정지

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>

#include <cmath>
#include <string>

namespace
{
inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- 파라미터/토픽 --------------------
static std::string g_topic_center_point;
static std::string g_motor_topic;
static std::string g_servo_topic;

static double g_bev_center_x_px  = 320.0;
static double g_target_offset_px = 80.0;  // 오른쪽 양수

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
static double g_run_duration_sec = 8.0;

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

  g_latest_steer_cmd = steer_cmd;
  g_latest_speed_cmd = speed_cmd;

  ROS_INFO_THROTTLE(0.5,
    "[ab_right] dx=%.1fpx err=%.3f steer=%.3f speed=%.2f",
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

void mission_ab_right_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  pnh.param<std::string>("topic_center_point",
                         g_topic_center_point,
                         std::string("/perception/center_point_px"));
  pnh.param<std::string>("motor_topic", g_motor_topic,
                         std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic", g_servo_topic,
                         std::string("/commands/servo/position"));

  pnh.param<double>("bev_center_x_px", g_bev_center_x_px, 320.0);
  pnh.param<double>("target_offset_px_right", g_target_offset_px, 80.0);

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
  pnh.param<double>("ab_right_run_duration_sec", g_run_duration_sec, 8.0);

  g_center_sub = nh.subscribe(g_topic_center_point, 20, centerCB);
  g_pub_motor  = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_pub_servo  = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);

  g_started = false;
  g_done = false;
  g_prev_steer = 0.0;
  g_have_cb_time = false;
  g_start_time = ros::Time(0);

  ROS_INFO("[ab_right] init: center='%s' motor='%s' servo='%s' offset=%.1fpx duration=%.1fs",
           ros::names::resolve(g_topic_center_point).c_str(),
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str(),
           g_target_offset_px, g_run_duration_sec);
}

void mission_ab_right_step()
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
    ROS_INFO("[ab_right] done (t=%.2fs)", t);
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

bool mission_ab_right_done()
{
  return g_done;
}

void mission_ab_right_reset()
{
  g_started = false;
  g_done = false;
  g_prev_steer = 0.0;
  g_have_cb_time = false;
  g_start_time = ros::Time(0);
}
