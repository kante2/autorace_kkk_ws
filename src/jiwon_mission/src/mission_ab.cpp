/******************************************************
 * mission_ab.cpp
 *  - 2초 진입(LEFT/RIGHT) 후 → lane_center_controller 로직 수행
 ******************************************************/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PointStamped.h>

#include <cmath>
#include <string>

inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

/******************************************************
 * 전역 상태
 ******************************************************/
std::string g_topic_center_point;
std::string g_motor_topic;
std::string g_servo_topic;
std::string g_dir_topic;

double g_bev_center_x_px = 320.0;

double g_servo_center = 0.5;
double g_servo_min    = 0.0;
double g_servo_max    = 1.0;
double g_steer_sign   = -1.0;

// dx bias (합류구간 보정용)
double g_dx_bias_px = 0.0;

// dx 기반 steer control params
double g_max_abs_dx_px = 83.0;
double g_dx_tolerance  = 3.0;
double g_alpha_ema     = 0.2;
double g_max_delta     = 0.08;
double g_steer_gain    = 1.5;

// speed plan
double g_base_speed_mps  = 7.0;
double g_min_speed_mps   = 0.8;
double g_speed_drop_gain = 0.5;

// motor scale
double g_motor_min_cmd = 0.0;
double g_motor_max_cmd = 900.0;
double g_motor_gain    = 300.0;

// 2초 진입 관련 변수
bool        g_in_turn_phase = false;
int         g_turn_dir = 0;           // -1=LEFT, +1=RIGHT
ros::Time   g_turn_start_time;
double      g_entry_duration_sec = 2.0;  // 진입 시간
double      g_entry_steer_cmd = 1.0;     // 진입 조향 세기

// 상태 저장
double g_latest_steer_cmd = 0.0;
double g_latest_speed_cmd = 0.0;
double g_prev_steer = 0.0;

bool g_have_cb_time = false;
ros::Time g_last_cb_time;
double g_dx_ema = 0.0;
bool   g_have_dx_ema = false;

// pubs
ros::Publisher g_pub_motor;
ros::Publisher g_pub_servo;

/******************************************************
 * processDx(dx)
 ******************************************************/
void processDx(double dx)
{
  double err_norm = (fabs(dx) <= g_dx_tolerance)
                      ? 0.0
                      : clamp(dx / g_max_abs_dx_px, -1.0, 1.0);

  double steer_raw = tanh(g_steer_gain * err_norm);

  double steer_smooth = g_alpha_ema * steer_raw +
                        (1.0 - g_alpha_ema) * g_prev_steer;

  double delta = clamp(steer_smooth - g_prev_steer,
                       -g_max_delta, g_max_delta);

  double steer_cmd = g_prev_steer + delta;
  g_prev_steer = steer_cmd;

  double speed_cmd = clamp(g_base_speed_mps -
                           g_speed_drop_gain * fabs(err_norm),
                           g_min_speed_mps, g_base_speed_mps);

  g_latest_steer_cmd = steer_cmd;
  g_latest_speed_cmd = speed_cmd;

  ROS_INFO_THROTTLE(0.3,
    "[lane] dx=%.1f, steer=%.3f, speed=%.2f",
    dx, steer_cmd, speed_cmd);
}

/******************************************************
 * centerPoint Callback
 ******************************************************/
void centerCB(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  g_last_cb_time = ros::Time::now();
  g_have_cb_time = true;

  double cx = msg->point.x;
  double dx_raw = cx - (g_bev_center_x_px + g_dx_bias_px);

  if (!g_have_dx_ema) {
    g_dx_ema = dx_raw;
    g_have_dx_ema = true;
  } else {
    g_dx_ema = 0.5 * dx_raw + 0.5 * g_dx_ema;
  }

  // ★ lane control 은 processDx 에서 처리
  processDx(g_dx_ema);
}

/******************************************************
 * 방향 명령 Callback (LEFT/RIGHT)
 ******************************************************/
void dirCB(const std_msgs::Int32::ConstPtr& msg)
{
  int d = msg->data;
  if (d == 0) {
    g_in_turn_phase = false;
    g_turn_dir = 0;
    g_dx_bias_px = 0.0; // 복귀
    ROS_INFO("[AB] dir=0 → turn phase 종료, dx_bias reset");
    return;
  }

  g_in_turn_phase = true;
  g_turn_dir = (d < 0) ? -1 : +1;
  g_turn_start_time = ros::Time::now();

  // 진입 직전에 line이 사라지는 구간 → 안쪽으로 bias 주기
  g_dx_bias_px = (g_turn_dir < 0 ? -40.0 : +40.0);

  ROS_INFO("[AB] Start turn dir=%d, bias=%.1fpx", g_turn_dir, g_dx_bias_px);
}

/******************************************************
 * main
 ******************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_ab_lane_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("mission_ab_lane_controller running...");

  // parameters
  pnh.param<std::string>("topic_center_point",
                         g_topic_center_point,
                         std::string("/perception/center_point_px"));
  pnh.param<std::string>("motor_topic",
                         g_motor_topic,
                         std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic",
                         g_servo_topic,
                         std::string("/commands/servo/position"));
  pnh.param<std::string>("dir_topic",
                         g_dir_topic,
                         std::string("/mission/ab_turn_dir"));

  pnh.param<double>("entry_duration_sec", g_entry_duration_sec, 2.0);
  pnh.param<double>("entry_steer_cmd",    g_entry_steer_cmd,    1.0);

  // subs
  ros::Subscriber sub_center =
      nh.subscribe(g_topic_center_point, 20, centerCB);
  ros::Subscriber sub_dir =
      nh.subscribe(g_dir_topic, 10, dirCB);

  // pubs
  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_pub_servo = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);

  ros::Rate rate(30);

  while (ros::ok())
  {
    ros::spinOnce();

    double steer_cmd = 0.0;
    double speed_cmd = 0.0;

    // =======================
    // 1) AB 진입 2초 구간
    // =======================
    if (g_in_turn_phase)
    {
      double t = (ros::Time::now() - g_turn_start_time).toSec();

      if (t < g_entry_duration_sec)
      {
        // 강제 조향
        steer_cmd = g_turn_dir * g_entry_steer_cmd;
        speed_cmd = 2.0;  // 진입 속도는 고정

        ROS_INFO_THROTTLE(0.3,
          "[AB] ENTRY t=%.2f/%.2f steer=%.2f",
          t, g_entry_duration_sec, steer_cmd);
      }
      else
      {
        // 2초 지나면 → 일반 lane 제어로 전환
        steer_cmd = g_latest_steer_cmd;
        speed_cmd = g_latest_speed_cmd;
      }
    }
    // =======================
    // 2) 일반 차선 주행
    // =======================
    else
    {
      steer_cmd = g_latest_steer_cmd;
      speed_cmd = g_latest_speed_cmd;
    }

    // servo 변환
    double steer_norm = clamp(steer_cmd, -1.0, 1.0);
    steer_norm *= (-g_steer_sign);

    double servo_range = std::min(g_servo_center - g_servo_min,
                                  g_servo_max - g_servo_center);

    double servo_hw = g_servo_center + steer_norm * servo_range;
    servo_hw = clamp(servo_hw, g_servo_min, g_servo_max);

    // motor
    double motor_cmd = g_motor_gain * speed_cmd;
    motor_cmd = clamp(motor_cmd, g_motor_min_cmd, g_motor_max_cmd);

    // pub
    std_msgs::Float64 m, s;
    m.data = motor_cmd;
    s.data = servo_hw;
    g_pub_motor.publish(m);
    g_pub_servo.publish(s);

    rate.sleep();
  }

  return 0;
}
