// mission_crosswalk_toMain.cpp
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string>
#include <cmath>

// -------------------- 유틸 --------------------
inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- 전역 상태 --------------------
// 토픽 이름
static std::string g_motor_topic;
static std::string g_servo_topic;

// 서보 스케일
static double g_servo_center = 0.5;
static double g_servo_min    = 0.0;
static double g_servo_max    = 1.0;
static double g_steer_sign   = -1.0;

// 속도/모터 스케일
static double g_go_speed_mps   = 2.0;   // 횡단보도 통과할 때 직진 속도
static double g_motor_min_cmd  = 0.0;
static double g_motor_max_cmd  = 1200.0;
static double g_motor_gain     = 500.0;

// 타이밍 (초)
static double g_stop_duration_sec = 7.0;  // 정지 시간
static double g_go_duration_sec   = 1.5;  // 직진 시간

// 퍼블리셔
static ros::Publisher g_pub_motor;
static ros::Publisher g_pub_servo;

// 내부 타이머 상태
static bool      g_crosswalk_running = false;
static ros::Time g_crosswalk_start_time;
static ros::Time g_last_step_time;

// -------------------- 초기화 함수 --------------------
void mission_crosswalk_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[crosswalk] mission_crosswalk_init()");

  // --- 파라미터 로드 ---
  pnh.param<std::string>("motor_topic", g_motor_topic,
                         std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic", g_servo_topic,
                         std::string("/commands/servo/position"));

  // 서보 스케일 (lane 미션과 동일하게 사용)
  pnh.param<double>("servo_center", g_servo_center, 0.5);
  pnh.param<double>("servo_min",    g_servo_min,    0.0);
  pnh.param<double>("servo_max",    g_servo_max,    1.0);
  pnh.param<double>("steer_sign",   g_steer_sign,  -1.0);

  // 속도/모터 스케일
  pnh.param<double>("crosswalk_go_speed_mps", g_go_speed_mps, 2.0);
  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);
  pnh.param<double>("motor_max_cmd", g_motor_max_cmd, 900.0);
  pnh.param<double>("motor_gain",    g_motor_gain,    300.0);

  // 타이밍
  pnh.param<double>("crosswalk_stop_duration_sec", g_stop_duration_sec, 7.0);
  pnh.param<double>("crosswalk_go_duration_sec",   g_go_duration_sec,   2.0);

  ROS_INFO("[crosswalk] publish motor='%s', servo='%s'",
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str());
  ROS_INFO("[crosswalk] stop=%.1fs, go=%.1fs, go_speed=%.2f m/s",
           g_stop_duration_sec, g_go_duration_sec, g_go_speed_mps);

  // --- Pub ---
  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_pub_servo = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);

  g_crosswalk_running = false;
  g_last_step_time    = ros::Time(0);

  ROS_INFO("[crosswalk] mission_crosswalk_init done");
}

// -------------------- 한 스텝 실행 --------------------
void mission_crosswalk_step()
{
  ros::Time now = ros::Time::now();

  // 오래 쉬었다가 다시 들어오면 리셋
  if (!g_last_step_time.isZero())
  {
    double gap = (now - g_last_step_time).toSec();
    if (gap > 0.5)  // 예: 0.5초 이상 호출 끊겼으면 새 미션으로 판단
    {
      g_crosswalk_running = false;
      ROS_INFO("[crosswalk] re-entered mission -> reset timer");
    }
  }
  g_last_step_time = now;

  // 처음 진입 시 타이머 시작
  if (!g_crosswalk_running)
  {
    g_crosswalk_running    = true;
    g_crosswalk_start_time = now;
    ROS_INFO("[crosswalk] START: stop %.1fs + go %.1fs",
             g_stop_duration_sec, g_go_duration_sec);
  }

  double elapsed = (now - g_crosswalk_start_time).toSec();

  double steer_cmd = 0.0;  // 직진
  double speed_cmd = 0.0;  // 아래 단계별 설정

  // ==============================
  // 0 ~ stop_duration      : 정지
  // stop_duration ~ stop+go : 직진
  // 그 이후                : 계속 직진 유지
  // ==============================
  if (elapsed < g_stop_duration_sec)
  {
    steer_cmd = 0.0;
    speed_cmd = 0.0;
    ROS_INFO_THROTTLE(1.0,
      "[crosswalk] STOP PHASE (t=%.2f/%.2f)", elapsed, g_stop_duration_sec);
  }
  else if (elapsed < g_stop_duration_sec + g_go_duration_sec)
  {
    steer_cmd = 0.0;
    speed_cmd = g_go_speed_mps;
    ROS_INFO_THROTTLE(1.0,
      "[crosswalk] GO PHASE (t=%.2f, %.2f~%.2f)",
      elapsed,
      g_stop_duration_sec,
      g_stop_duration_sec + g_go_duration_sec);
  }
  else
  {
    steer_cmd = 0.0;
    speed_cmd = g_go_speed_mps;
    ROS_INFO_THROTTLE(2.0,
      "[crosswalk] DONE (elapsed=%.2f) -> keep going straight until main switches mission",
      elapsed);
  }

  // ---- 1) 조향 변환: -1~+1 -> 서보 0~1 ----
  double steer_norm = clamp(steer_cmd, -1.0, 1.0);

  // 방향 뒤집기(차에 맞게)
  steer_norm *= (-g_steer_sign);

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
    "[crosswalk][loop] t=%.2f steer=%.3f servo=%.3f motor=%.1f v=%.2f",
    elapsed, steer_cmd, servo_hw, motor_cmd, speed_cmd);
}
