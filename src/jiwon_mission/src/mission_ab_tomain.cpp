// mission_AB_toMain.cpp
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string>
#include <cmath>
#include <std_msgs/Int32.h>

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
static double g_turn_speed_mps = 2.0;   // 회전하면서 전진 속도
static double g_motor_min_cmd  = 0.0;
static double g_motor_max_cmd  = 1000.0;
static double g_motor_gain     = 300.0;

// 회전 관련 파라미터
static double g_turn_duration_sec = 2.0;   // 회전+전진 시간 (초)
static double g_turn_steer_cmd    = 0.5;   // -1~+1 중 어느 정도로 꺾을지

// 퍼블리셔
static ros::Publisher g_pub_motor;
static ros::Publisher g_pub_servo;

// 내부 타이머 상태
static bool      g_turn_running     = false;
static ros::Time g_turn_start_time;
static ros::Time g_last_step_time;

// 현재 진행 중인 방향 (이전 dir 기억용)
static int g_current_turn_dir = 0;

// -------------------- 초기화 함수 --------------------
void mission_AB_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[AB_turn] mission_AB_init()");

  // --- 파라미터 로드 ---
  pnh.param<std::string>("motor_topic", g_motor_topic,
                         std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic", g_servo_topic,
                         std::string("/commands/servo/position"));

  // 서보 스케일
  pnh.param<double>("servo_center", g_servo_center, 0.5);
  pnh.param<double>("servo_min",    g_servo_min,    0.0);
  pnh.param<double>("servo_max",    g_servo_max,    1.0);
  pnh.param<double>("steer_sign",   g_steer_sign,  -1.0);

  // 속도/모터 스케일
  pnh.param<double>("turn_speed_mps", g_turn_speed_mps, 2.0);
  pnh.param<double>("motor_min_cmd",  g_motor_min_cmd,  0.0);
  pnh.param<double>("motor_max_cmd",  g_motor_max_cmd,  1000.0);
  pnh.param<double>("motor_gain",     g_motor_gain,     300.0);

  // 회전 관련 파라미터
  pnh.param<double>("turn_duration_sec", g_turn_duration_sec, 2.0);
  pnh.param<double>("turn_steer_cmd",    g_turn_steer_cmd,    0.5);

  ROS_INFO("[AB_turn] publish motor='%s', servo='%s'",
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_servo_topic).c_str());
  ROS_INFO("[AB_turn] turn_duration=%.1fs, speed=%.2f m/s, steer_cmd=%.2f",
           g_turn_duration_sec, g_turn_speed_mps, g_turn_steer_cmd);

  // --- Pub ---
  g_pub_motor = nh.advertise<std_msgs::Float64>(g_motor_topic, 10);
  g_pub_servo = nh.advertise<std_msgs::Float64>(g_servo_topic, 10);

  g_turn_running     = false;
  g_turn_start_time  = ros::Time(0);
  g_last_step_time   = ros::Time(0);
  g_current_turn_dir = 0;

  ROS_INFO("[AB_turn] mission_AB_init done");
}

// -------------------- 한 스텝 실행 --------------------
// dir: -1 = LEFT, +1 = RIGHT, 0 = NONE
void mission_AB_step(int dir)
{
  ros::Time now = ros::Time::now();

  // 오래 쉬었다가 다시 들어오면 리셋
  if (!g_last_step_time.isZero())
  {
    double gap = (now - g_last_step_time).toSec();
    if (gap > 0.5)
    {
      g_turn_running     = false;
      g_turn_start_time  = ros::Time(0);
      g_current_turn_dir = 0;
      ROS_INFO("[AB_turn] re-entered mission -> reset timer");
    }
  }
  g_last_step_time = now;

  // dir이 0이면: 회전 미션이 아니라는 뜻 → 여기서는 정지
  if (dir == 0)
  {
    double steer_cmd = 0.0;
    double speed_cmd = 0.0;

    double steer_norm = clamp(steer_cmd, -1.0, 1.0);
    steer_norm *= (-g_steer_sign);

    double servo_range = std::min(g_servo_center - g_servo_min,
                                  g_servo_max - g_servo_center);
    double servo_hw = g_servo_center + steer_norm * servo_range;
    servo_hw = clamp(servo_hw, g_servo_min, g_servo_max);

    double motor_cmd = g_motor_gain * speed_cmd;
    motor_cmd = clamp(motor_cmd, g_motor_min_cmd, g_motor_max_cmd);

    std_msgs::Float64 motor_msg;
    std_msgs::Float64 servo_msg;
    motor_msg.data = motor_cmd;
    servo_msg.data = servo_hw;

    g_pub_motor.publish(motor_msg);
    g_pub_servo.publish(servo_msg);

    ROS_INFO_THROTTLE(1.0,
      "[AB_turn] dir=0 -> stop");
    return;
  }

  // 방향이 바뀌었거나, 처음 시작하는 경우 → 타이머 리셋 후 시작
  if (!g_turn_running || (dir != g_current_turn_dir))
  {
    g_turn_running     = true;
    g_turn_start_time  = now;
    g_current_turn_dir = dir;

    ROS_INFO("[AB_turn] START: dir=%d, duration=%.1fs, speed=%.2f",
             g_current_turn_dir, g_turn_duration_sec, g_turn_speed_mps);
  }

  double elapsed = (now - g_turn_start_time).toSec();

  double steer_cmd = 0.0;
  double speed_cmd = 0.0;

  if (elapsed < g_turn_duration_sec)
  {
    // duration 동안 해당 방향으로 회전 + 전진
    steer_cmd = g_current_turn_dir * g_turn_steer_cmd;  // -0.5 or +0.5
    speed_cmd = g_turn_speed_mps;

    ROS_INFO_THROTTLE(0.5,
      "[AB_turn] TURN PHASE t=%.2f/%.2f dir=%d steer=%.2f v=%.2f",
      elapsed, g_turn_duration_sec, g_current_turn_dir, steer_cmd, speed_cmd);
  }
  else
  {
    // 회전 끝: 직진 유지 (필요하면 speed_cmd=0.0로 바꿔도 가능)
    steer_cmd = 1000.0;
    //steer_cmd = 0;
    speed_cmd = g_turn_speed_mps;

    ROS_INFO_THROTTLE(1.0,
      "[AB_turn] DONE (elapsed=%.2f) -> go straight (v=%.2f)",
      elapsed, speed_cmd);
  }

  // ---- 1) 조향 변환: -1~+1 -> 서보 0~1 ----
  double steer_norm = clamp(steer_cmd, -1.0, 1.0);
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
    "[AB_turn][loop] t=%.2f dir=%d steer_cmd=%.3f servo=%.3f motor=%.1f v=%.2f",
    elapsed, g_current_turn_dir, steer_cmd, servo_hw, motor_cmd, speed_cmd);
}


// -------------------- dir 명령 수신용 전역 --------------------
static int g_latest_dir = 0;   // -1 = LEFT, 0 = NONE, +1 = RIGHT

void dirCallback(const std_msgs::Int32::ConstPtr& msg)
{
  int d = msg->data;
  if (d < 0)      g_latest_dir = -1;
  else if (d > 0) g_latest_dir = +1;
  else            g_latest_dir = 0;
}

// -------------------- main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_ab_turn_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 미션 초기화 (파라미터 로드 + pub 준비)
  mission_AB_init(nh, pnh);

  // dir 토픽 이름 파라미터 (원하면 바꿔써)
  std::string dir_topic;
  pnh.param<std::string>("dir_topic", dir_topic,
                         std::string("/mission/ab_turn_dir"));

  ros::Subscriber sub_dir = nh.subscribe(dir_topic, 10, dirCallback);

  ros::Rate rate(50.0);   // 50 Hz 루프

  ROS_INFO("[AB_turn] main loop started. Subscribed dir='%s'",
           ros::names::resolve(dir_topic).c_str());

  while (ros::ok())
  {
    ros::spinOnce();

    // 최신 dir 값으로 한 스텝 실행
    mission_AB_step(g_latest_dir);

    rate.sleep();
  }

  return 0;
}
