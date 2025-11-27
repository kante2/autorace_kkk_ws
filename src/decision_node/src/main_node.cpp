// main_node.cpp
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>

// -------------------- 미션 함수 선언 --------------------
// mission_lane.cpp
void mission_lane_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_lane_step();

// mission_labacorn.cpp
void mission_labacorn_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_labacorn_step();

// mission_gate.cpp
void mission_gate_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_gate_step();

// mission_crosswalk.cpp
void mission_crosswalk_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_crosswalk_step();

// missin_rotary.cpp
void mission_rotary_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_rotary_step();

// missin_parking.cpp
void mission_parking_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_parking_step();

// -------------------- 미션 상태 enum --------------------
enum MissionState
{
  MISSION_LANE = 0,
  MISSION_LABACORN,
  MISSION_GATE,
  MISSION_CROSSWALK,
  MISSION_ROTARY,
  MISSION_PARKING
};

MissionState g_current_state = MISSION_LANE;

// 감지 토픽 상태
bool g_labacorn_detected   = false;
bool g_gate_detected       = false;
bool g_crosswalk_detected  = false;
bool g_rotary_detected     = false;
bool g_parking_detected    = false;
bool g_parking_bag_lock    = false;
int  g_labacorn_count = 0;
bool g_labacorn_session_active = false;
ros::Time g_labacorn_on_start_time;
ros::Time g_labacorn_last_seen;
// const double k_labacorn_grace_sec = 5.0;        // 라바콘 끊김 완충/타임아웃 ** 로직에는 미완,// 먼저 라바콘 도중 끊킴이 없으면 해당 조건문을 추가하지 않아도됨
// rotary 타이밍 제어
const double k_rotary_hold_sec = 8.0;           // 코칼콘 8초 로직 유지
bool g_rotary_hold_active = false;
ros::Time g_rotary_hold_start;

// labacorn 감지 안정화 (히스테리시스)
const double k_labacorn_confirm_sec  = 0.5;     // 이 시간 연속 감지 시에만 미션 진입
const double k_labacorn_release_sec = 0.5;      // 이 시간 이상 미감지 시 미션 종료
bool g_labacorn_confirmed = false;              // 안정화된 라바콘 감지 플래그


// -------------------- 콜백: 라바콘 감지 --------------------
void CB_LabacornDetected(const std_msgs::Bool::ConstPtr& msg)
{
  g_labacorn_detected = msg->data;
  if (msg->data)
  {
    g_labacorn_last_seen = ros::Time::now();
  }
}

// -------------------- 콜백: 게이트 감지 --------------------
void CB_GateDetected(const std_msgs::Bool::ConstPtr& msg)
{
  g_gate_detected = msg->data;
}

// -------------------- 콜백: 횡단보도 감지 --------------------
void CB_CrosswalkDetected(const std_msgs::Bool::ConstPtr& msg)
{
  g_crosswalk_detected = msg->data;
}

// -------------------- 콜백: rotary 감지 --------------------
void CB_RotaryDetected(const std_msgs::Bool::ConstPtr& msg)
{
  g_rotary_detected = msg->data;
}

void CB_ParkingDetected(const std_msgs::Bool::ConstPtr& msg)
{
  g_parking_detected = msg->data;
}

void CB_ParkingBagLock(const std_msgs::Bool::ConstPtr& msg)
{
  g_parking_bag_lock = msg->data;
}

// -------------------- 콜백: AB 좌/우 감지 --------------------
// -------------------- main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "autorace_main_decision");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("[main_node] autorace_decision main_node started");

  // ===== 감지 토픽 이름 파라미터 =====
  std::string topic_labacorn_detected;
  std::string topic_gate_detected;
  std::string topic_crosswalk_detected;
  std::string topic_rotary_detected;
  std::string topic_parking_detected;
  std::string topic_parking_bag_lock;

  pnh.param<std::string>("labacorn_detected_topic", topic_labacorn_detected, std::string("/labacorn_detected"));
  pnh.param<std::string>("gate_detected_topic", topic_gate_detected, std::string("/gate_detected"));
  pnh.param<std::string>("crosswalk_detected_topic", topic_crosswalk_detected, std::string("/crosswalk_detected"));
  pnh.param<std::string>("rotary_detected_topic",topic_rotary_detected,std::string("/rotary_detected"));
  pnh.param<std::string>("parking_detected_topic",topic_parking_detected,std::string("/parking_detected"));
  pnh.param<std::string>("parking_bag_lock_topic", topic_parking_bag_lock,std::string("/parking_bag_lock"));

  // ===== 감지 토픽 구독 =====
  ros::Subscriber sub_labacorn =  nh.subscribe(topic_labacorn_detected, 1, CB_LabacornDetected);
  ros::Subscriber sub_gate =      nh.subscribe(topic_gate_detected, 1, CB_GateDetected);
  ros::Subscriber sub_crosswalk = nh.subscribe(topic_crosswalk_detected, 1, CB_CrosswalkDetected);
  ros::Subscriber sub_rotary =    nh.subscribe(topic_rotary_detected, 1, CB_RotaryDetected);
  ros::Subscriber sub_parking =   nh.subscribe(topic_parking_detected, 1, CB_ParkingDetected);
  ros::Subscriber sub_parking_lock = nh.subscribe(topic_parking_bag_lock, 1, CB_ParkingBagLock);

  // ===== 각 미션 초기화 =====
  mission_lane_init(nh, pnh);
  mission_labacorn_init(nh, pnh);
  mission_gate_init(nh, pnh);
  mission_crosswalk_init(nh, pnh);
  mission_rotary_init(nh, pnh);
  mission_parking_init(nh, pnh);

  ROS_INFO("[main_node] all mission init done");

  ros::Rate rate(10.0);  // 10 Hz

  MissionState prev_state = g_current_state;

  while (ros::ok())
  {
    ros::spinOnce();
    const ros::Time now = ros::Time::now();

    // 라바콘 감지 히스테리시스 (연속 on/off 시간 요구)
    if (g_labacorn_detected)
    {
      if (g_labacorn_on_start_time.isZero())
      {
        g_labacorn_on_start_time = now;
      }
      double on_elapsed = (now - g_labacorn_on_start_time).toSec();
      if (on_elapsed >= k_labacorn_confirm_sec)
      {
        g_labacorn_confirmed = true;
      }
      g_labacorn_last_seen = now;
    }
    else
    {
      if (!g_labacorn_last_seen.isZero())
      {
        double off_elapsed = (now - g_labacorn_last_seen).toSec();
        if (off_elapsed >= k_labacorn_release_sec)
        {
          g_labacorn_confirmed = false;
          g_labacorn_on_start_time = ros::Time(0);
        }
      }
      else
      {
        g_labacorn_confirmed = false;
      }
    }

    // rotary 진입 후 8초간 모드 유지
    bool rotary_hold_now = false;
    if (g_rotary_hold_active)
    {
      double hold_elapsed = (now - g_rotary_hold_start).toSec();
      if (hold_elapsed < k_rotary_hold_sec) rotary_hold_now = true;
      else
      {
        g_rotary_hold_active = false;
        ROS_INFO("[main_node] ROTARY hold finished");
      }
    }

    // 디버깅: 상태 결정 전에 주요 플래그 로그 (1초 주기)
    // ** 
    ROS_INFO_THROTTLE(1.0,
                      "[main_node] flags lane debug | "
                      "labacorn=%d gate=%d crosswalk=%d rotary=%d parking=%d bag_lock=%d",
                      (int)g_labacorn_detected, (int)g_gate_detected, (int)g_crosswalk_detected,
                      (int)g_rotary_detected, (int)g_parking_detected, (int)g_parking_bag_lock);

    // -----------------------------
    // 1) 미션 상태 결정 로직
    //    (우선순위: Parking bag lock > Crosswalk > Gate > Rotary > Labacorn(안정화) > Parking > Lane)
    // -----------------------------
    if (g_parking_bag_lock)                                 g_current_state = MISSION_PARKING;
    else if (g_crosswalk_detected)                          g_current_state = MISSION_CROSSWALK;
    else if (g_gate_detected)                               g_current_state = MISSION_GATE;
    else if (rotary_hold_now || g_rotary_detected)          g_current_state = MISSION_ROTARY;
    else if (g_labacorn_confirmed)                          g_current_state = MISSION_LABACORN;
    // else if (g_parking_detected)                            g_current_state = MISSION_PARKING;
    else                                                    g_current_state = MISSION_LANE;

    // 상태 변경 시 로그
    if (g_current_state != prev_state)
    {
      const char* state_name = "LANE";
      if (g_current_state == MISSION_LABACORN)        state_name = "LABACORN";
      else if (g_current_state == MISSION_GATE)       state_name = "GATE";
      else if (g_current_state == MISSION_CROSSWALK)  state_name = "CROSSWALK";
      else if (g_current_state == MISSION_ROTARY)     state_name = "ROTARY";
      else if (g_current_state == MISSION_PARKING)    state_name = "PARKING";

      ROS_INFO("[main_node] Mission changed -> %s", state_name);
      if (prev_state == MISSION_LABACORN && g_current_state != MISSION_LABACORN) // 라바콘 빠져나가는 부분 디버깅용 ** 
      {
        ROS_WARN("[main_node] LABACORN exit -> %s (labacorn_detected=%d)",state_name, (int)g_labacorn_detected);
      }
      if (g_current_state == MISSION_ROTARY)
      {
        g_rotary_hold_active = true;
        g_rotary_hold_start = now;
        ROS_INFO("[main_node] ROTARY entered -> hold for %.1fs", k_rotary_hold_sec);
      }

      if (g_current_state == MISSION_LABACORN)
      {
        g_labacorn_count++;
        ROS_INFO("[main_node] LABACORN entry #%d (confirmed detect)", g_labacorn_count);
      }

      // -------------------------------------

      prev_state = g_current_state;
    }

    // -----------------------------
    // 2) 현재 상태에 맞는 미션 한 스텝 실행
    // -----------------------------
    switch (g_current_state)
    {
      case MISSION_LANE:
        ROS_DEBUG("MISSION_LANE mode");
        mission_lane_step();
        break;

      case MISSION_CROSSWALK:
        ROS_DEBUG("MISSION_CROSSWALK mode changed ");
        mission_crosswalk_step();   // ← 여기 안에서 7초 정지 + 2초 직진
        break;

      case MISSION_LABACORN:
        ROS_DEBUG("MISSION_LABACORN mode changed");
        // 라바콘, 터널 둘 다 이 로직 사용
        mission_labacorn_step();
        break;

      case MISSION_GATE:
        ROS_DEBUG("MISSION_GATE mode changed ");
        mission_gate_step();
        break;
      
      case MISSION_ROTARY:
        ROS_DEBUG("MISSION_ROTARY mode changed");
        mission_rotary_step();
        break;

      case MISSION_PARKING:
        ROS_DEBUG("MISSION_PARKING mode changed");
        mission_parking_step();
        break;

      default:
        ROS_DEBUG("MISSION_Lane mode changed");
        mission_lane_step();
        break;
    }

    rate.sleep();
  }
  return 0;
}
