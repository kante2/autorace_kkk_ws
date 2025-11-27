// mission_node.cpp
// 센서(카메라/라이다) enable 신호를 타이머로 순차적으로 내보내고,
// 지정된 구간에서는 하드코딩 직진 명령을 퍼블리시한다.

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <algorithm>
#include <vector>
#include <string>

enum class PhaseKind
{
  CAMERA_ONLY,
  BAG_ONLY,
  LIDAR_ONLY,
  HARD_DRIVE
};

struct Phase
{
  std::string name;
  PhaseKind kind;
  double duration;  // seconds
};

struct MissionNode
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  // enable pubs (latched)
  ros::Publisher pub_camera_enable;
  ros::Publisher pub_lidar_enable;
  ros::Publisher pub_bag_enable;

  // perception fan-out
  ros::Publisher pub_crosswalk_enable;
  ros::Publisher pub_ab_enable;
  ros::Publisher pub_ab_sign_enable;
  ros::Publisher pub_gate_enable;
  ros::Publisher pub_rotary_enable;
  ros::Publisher pub_parking_enable;

  // hard-drive command pubs
  ros::Publisher pub_motor;
  ros::Publisher pub_servo;

  ros::Timer timer;

  std::vector<Phase> phases;
  std::size_t current_idx{0};
  ros::Time phase_start;
  bool hard_active{false};

  double hard_motor{1500.0};
  double hard_servo{0.567};
  double hard_duration{8.0};
  double timer_hz{20.0};

  MissionNode() : pnh("~")
  {
    // 토픽 이름 파라미터화
    const std::string camera_enable_topic = pnh.param<std::string>("camera_enable_topic", "/mission/camera_enable");
    const std::string lidar_enable_topic  = pnh.param<std::string>("lidar_enable_topic", "/mission/lidar_enable");
    const std::string bag_enable_topic    = pnh.param<std::string>("bag_enable_topic", "/mission/bag_enable");

    const std::string crosswalk_enable_topic = pnh.param<std::string>("crosswalk_enable_topic", "/perception/crosswalk/enable");
    const std::string ab_enable_topic        = pnh.param<std::string>("ab_enable_topic", "/perception/ab/enable");
    const std::string ab_sign_enable_topic   = pnh.param<std::string>("ab_sign_enable_topic", "/ab_sign/enable");
    const std::string gate_enable_topic      = pnh.param<std::string>("gate_enable_topic", "/perception/gate/enable");
    const std::string rotary_enable_topic    = pnh.param<std::string>("rotary_enable_topic", "/perception/rotary/enable");
    const std::string parking_enable_topic   = pnh.param<std::string>("parking_enable_topic", "/perception/parking/enable");

    const std::string servo_topic = pnh.param<std::string>("servo_topic", "/commands/servo/position");
    const std::string motor_topic = pnh.param<std::string>("motor_topic", "/commands/motor/speed");

    // 기본 파라미터
    const double camera1_sec = pnh.param<double>("camera1_sec", 20.0);
    const double bag_sec     = pnh.param<double>("bag_sec", 5.0);
    const double lidar1_sec  = pnh.param<double>("lidar1_sec", 8.0);
    const double camera2_sec = pnh.param<double>("camera2_sec", 3.0);
    const double lidar2_sec  = pnh.param<double>("lidar2_sec", 8.0);
    const double camera3_sec = pnh.param<double>("camera3_sec", 3.0);
    hard_duration            = pnh.param<double>("hard_sec", 8.0);
    const double camera4_sec = pnh.param<double>("camera4_sec", 4.0);
    const double lidar3_sec  = pnh.param<double>("lidar3_sec", 3.0);
    const double camera5_sec = pnh.param<double>("camera5_sec", 10.0);

    hard_motor = pnh.param<double>("hard_motor_speed", 1500.0);
    hard_servo = pnh.param<double>("hard_servo_center", 0.567);
    timer_hz   = pnh.param<double>("timer_hz", 20.0);

    // enable publishers (latched so 늦게 붙어도 상태 유지)
    pub_camera_enable   = nh.advertise<std_msgs::Bool>(camera_enable_topic, 1, true);
    pub_lidar_enable    = nh.advertise<std_msgs::Bool>(lidar_enable_topic, 1, true);
    pub_bag_enable      = nh.advertise<std_msgs::Bool>(bag_enable_topic, 1, true);
    pub_crosswalk_enable= nh.advertise<std_msgs::Bool>(crosswalk_enable_topic, 1, true);
    pub_ab_enable       = nh.advertise<std_msgs::Bool>(ab_enable_topic, 1, true);
    pub_ab_sign_enable  = nh.advertise<std_msgs::Bool>(ab_sign_enable_topic, 1, true);
    pub_gate_enable     = nh.advertise<std_msgs::Bool>(gate_enable_topic, 1, true);
    pub_rotary_enable   = nh.advertise<std_msgs::Bool>(rotary_enable_topic, 1, true);
    pub_parking_enable  = nh.advertise<std_msgs::Bool>(parking_enable_topic, 1, true);

    pub_motor = nh.advertise<std_msgs::Float64>(motor_topic, 1);
    pub_servo = nh.advertise<std_msgs::Float64>(servo_topic, 1);

    // 단계 구성 (요청한 순서)
    phases = {
      {"CAMERA_ONLY_1", PhaseKind::CAMERA_ONLY, camera1_sec},
      {"BAG_ONLY_1",    PhaseKind::BAG_ONLY,     bag_sec},
      {"BAG_ONLY_2",    PhaseKind::BAG_ONLY,     bag_sec},
      {"LIDAR_ONLY_1",  PhaseKind::LIDAR_ONLY,   lidar1_sec},
      {"CAMERA_ONLY_2", PhaseKind::CAMERA_ONLY,  camera2_sec},
      {"LIDAR_ONLY_2",  PhaseKind::LIDAR_ONLY,   lidar2_sec},
      {"CAMERA_ONLY_3", PhaseKind::CAMERA_ONLY,  camera3_sec},
      {"HARD_DRIVE",    PhaseKind::HARD_DRIVE,   hard_duration},
      {"CAMERA_ONLY_4", PhaseKind::CAMERA_ONLY,  camera4_sec},
      {"LIDAR_ONLY_3",  PhaseKind::LIDAR_ONLY,   lidar3_sec},
      {"CAMERA_ONLY_5", PhaseKind::CAMERA_ONLY,  camera5_sec},
    };

    phase_start = ros::Time::now();
    applyPhase(phases.at(current_idx));

    const double timer_period = 1.0 / std::max(1.0, timer_hz);
    timer = nh.createTimer(ros::Duration(timer_period), &MissionNode::onTimer, this);
    ROS_INFO("[mission_node] ready with %zu phases, timer=%.2f Hz", phases.size(), 1.0 / timer_period);
  }

  void publishEnableSet(bool camera_on, bool lidar_on, bool bag_on)
  {
    std_msgs::Bool msg;

    msg.data = camera_on;
    pub_camera_enable.publish(msg);
    pub_crosswalk_enable.publish(msg);
    pub_ab_enable.publish(msg);
    pub_ab_sign_enable.publish(msg);

    msg.data = lidar_on;
    pub_lidar_enable.publish(msg);
    pub_gate_enable.publish(msg);
    pub_rotary_enable.publish(msg);
    pub_parking_enable.publish(msg);

    msg.data = bag_on;
    pub_bag_enable.publish(msg);
  }

  void publishHardDrive()
  {
    std_msgs::Float64 motor_msg;
    std_msgs::Float64 servo_msg;
    motor_msg.data = hard_motor;
    servo_msg.data = hard_servo;
    pub_motor.publish(motor_msg);
    pub_servo.publish(servo_msg);
  }

  void stopHardDrive()
  {
    if (!hard_active) return;
    std_msgs::Float64 motor_msg;
    motor_msg.data = 0.0;
    pub_motor.publish(motor_msg);
    hard_active = false;
  }

  void applyPhase(const Phase& phase)
  {
    bool cam_on = false;
    bool lidar_on = false;
    bool bag_on = false;
    bool hard_on = false;

    switch (phase.kind)
    {
      case PhaseKind::CAMERA_ONLY:
        cam_on = true;
        break;
      case PhaseKind::BAG_ONLY:
        bag_on = true;
        break;
      case PhaseKind::LIDAR_ONLY:
        lidar_on = true;
        break;
      case PhaseKind::HARD_DRIVE:
        cam_on = false;
        lidar_on = false;
        hard_on = true;
        break;
    }

    publishEnableSet(cam_on, lidar_on, bag_on);
    hard_active = hard_on;

    ROS_INFO("[mission_node] Phase -> %s (duration=%.1fs, cam=%d, lidar=%d, bag=%d, hard=%d)",
             phase.name.c_str(), phase.duration, cam_on, lidar_on, bag_on, hard_on);
  }

  void nextPhase()
  {
    stopHardDrive();
    current_idx++;
    if (current_idx >= phases.size())
    {
      ROS_INFO("[mission_node] all phases finished");
      publishEnableSet(false, false, false);
      return;
    }
    phase_start = ros::Time::now();
    applyPhase(phases.at(current_idx));
  }

  void onTimer(const ros::TimerEvent&)
  {
    if (current_idx >= phases.size()) return;

    const Phase& phase = phases[current_idx];
    const double elapsed = (ros::Time::now() - phase_start).toSec();
    if (elapsed >= phase.duration)
    {
      nextPhase();
      return;
    }

    if (phase.kind == PhaseKind::HARD_DRIVE)
    {
      publishHardDrive();  // keep commanding during hard step
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_node");
  MissionNode node;
  ros::spin();
  return 0;
}
