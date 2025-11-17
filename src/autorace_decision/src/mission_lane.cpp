#include "autorace_control/controller_lane_logic.hpp"

// -------------------- 전역 실제 정의 --------------------
// (여기서는 extern으로 선언된 애들 실제 메모리 자리 만들어 주는 부분)

std::string g_topic_center_point;
std::string g_topic_dx_px;
std::string g_motor_topic;
std::string g_servo_topic;
std::string g_is_crosswalk_topic;
std::string g_topic_curvature_center;

double g_bev_center_x_px = 320.0;

double g_servo_center = 0.5;
double g_servo_min    = 0.0;
double g_servo_max    = 1.0;
double g_steer_sign   = -1.0;

double g_max_abs_dx_px = 83.0;
double g_dx_tolerance  = 3.0;
double g_steer_gain    = 1.5;
double g_alpha_ema     = 0.2;
double g_max_delta     = 0.08;

double g_base_speed_mps  = 7.0;
double g_min_speed_mps   = 0.8;
double g_speed_drop_gain = 0.5;

double g_motor_min_cmd = 0.0;
double g_motor_max_cmd = 900.0;
double g_motor_gain    = 300.0;

double    g_dx_timeout_sec = 1.0;

ros::Publisher g_pub_motor;
ros::Publisher g_pub_servo;

double    g_prev_steer       = 0.0;
double    g_latest_steer_cmd = 0.0;
double    g_latest_speed_cmd = 0.0;
ros::Time g_last_cb_time;
bool      g_have_cb_time     = false;

bool      g_is_crosswalk            = false;
bool      g_crosswalk_timer_running = false;
ros::Time g_crosswalk_start_time;

double g_min_curv = 3e-4;
double g_max_curv = 1.5e-3;


// -------------------- main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_lane");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("[mission_lane] (center_point_px -> motor/servo Float64)");

  // 파라미터 로드 (logic cpp에 구현)
  loadParams(pnh);

  ROS_INFO("[lane_ctrl] subscribe center='%s'",
           ros::names::resolve(g_topic_center_point).c_str());
  ROS_INFO("[lane_ctrl] subscribe is_crosswalk='%s'",
           ros::names::resolve(g_is_crosswalk_topic).c_str());
  ROS_INFO("[lane_ctrl] subscribe curvature_center='%s'",
           ros::names::resolve(g_topic_curvature_center).c_str());
  ROS_INFO("[lane_ctrl] publish motor='%s', servo='%s'",
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str());

  // --- Pub/Sub ---
  ros::Subscriber center_sub     = nh.subscribe(g_topic_center_point,       20, CB_center);
  ros::Subscriber crosswalk_sub  = nh.subscribe(g_is_crosswalk_topic,       10, CB_crosswalk);
  ros::Subscriber curvature_sub  = nh.subscribe(g_topic_curvature_center,   10, CB_center_curvature);

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
      steer_cmd = g_latest_steer_cmd;
      speed_cmd = g_latest_speed_cmd;
    } else {
      steer_cmd = 0.0;
      speed_cmd = 0.0;
      ROS_INFO_THROTTLE(1.0,
        "[lane_ctrl] waiting /perception/center_point_px ... (timeout)");
    }

    // 1. crosswalk 7초 정지 로직 (그대로 유지)
    apply_crosswalk_hold(steer_cmd, speed_cmd, now);

    // 2. 조향/속도 -> 서보/모터 명령 변환
    publish_motor_commands(double steer_cmd, double speed_cmd);

    ROS_INFO_THROTTLE(0.5,
      "[lane_ctrl][loop] have_dx=%d steer_cmd=%.3f servo=%.3f motor=%.1f v=%.2f (steer_gain=%.3f)",
      (int)have_dx, steer_cmd, servo_hw, motor_cmd, speed_cmd, g_steer_gain);

    rate.sleep();
  }

  return 0;
}
