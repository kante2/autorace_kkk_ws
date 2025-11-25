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

// mission_ab
void mission_ab_left_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_ab_left_step();
bool mission_ab_left_done();
void mission_ab_left_reset();

void mission_ab_right_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_ab_right_step();
bool mission_ab_right_done();
void mission_ab_right_reset();

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
  MISSION_PARKING,
  MISSION_AB  // AB 표지판 좌/우 삽입 동작
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
bool g_ab_left_detected    = false;
bool g_ab_right_detected   = false;
int  g_ab_latched_dir      = -1; // -1 none, 0 left, 1 right
bool g_ab_action_running   = false;


// -------------------- 콜백: 라바콘 감지 --------------------
void CB_LabacornDetected(const std_msgs::Bool::ConstPtr& msg)
{
  g_labacorn_detected = msg->data;
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
void CB_AB_Left(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data) {
    g_ab_left_detected = true;
  }
}

void CB_AB_Right(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data) {
    g_ab_right_detected = true;
  }
}

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
  std::string topic_ab_enable;
  std::string topic_ab_left;
  std::string topic_ab_right;

  pnh.param<std::string>("labacorn_detected_topic",
                         topic_labacorn_detected,
                         std::string("/labacorn_detected"));

  pnh.param<std::string>("gate_detected_topic",
                         topic_gate_detected,
                         std::string("/gate_detected"));

  pnh.param<std::string>("crosswalk_detected_topic",
                         topic_crosswalk_detected,
                         std::string("/crosswalk_detected"));

  pnh.param<std::string>("rotary_detected_topic",
                         topic_rotary_detected,
                         std::string("/rotary_detected"));

  pnh.param<std::string>("parking_detected_topic",
                         topic_parking_detected,
                         std::string("/parking_detected"));
  pnh.param<std::string>("parking_bag_lock_topic",
                         topic_parking_bag_lock,
                         std::string("/parking_bag_lock"));
  pnh.param<std::string>("ab_enable_topic",
                         topic_ab_enable,
                         std::string("/ab_sign/enable"));
  pnh.param<std::string>("ab_left_topic",
                         topic_ab_left,
                         std::string("/ab_sign/left"));
  pnh.param<std::string>("ab_right_topic",
                         topic_ab_right,
                         std::string("/ab_sign/right"));

  ROS_INFO("[main_node] subscribe labacorn='%s', gate='%s', crosswalk='%s', rotary='%s', parking='%s'",
           ros::names::resolve(topic_labacorn_detected).c_str(),
           ros::names::resolve(topic_gate_detected).c_str(),
           ros::names::resolve(topic_crosswalk_detected).c_str(),
           ros::names::resolve(topic_rotary_detected).c_str(),
           ros::names::resolve(topic_parking_detected).c_str());
  ROS_INFO("[main_node] subscribe parking_bag_lock='%s'",
           ros::names::resolve(topic_parking_bag_lock).c_str());
  ROS_INFO("[main_node] publish ab_enable='%s'",
           ros::names::resolve(topic_ab_enable).c_str());
  ROS_INFO("[main_node] subscribe ab_left='%s', ab_right='%s'",
           ros::names::resolve(topic_ab_left).c_str(),
           ros::names::resolve(topic_ab_right).c_str());

  // ===== 감지 토픽 구독 =====
  ros::Subscriber sub_labacorn =
      nh.subscribe(topic_labacorn_detected, 1, CB_LabacornDetected);
  ros::Subscriber sub_gate =
      nh.subscribe(topic_gate_detected, 1, CB_GateDetected);
  ros::Subscriber sub_crosswalk =
      nh.subscribe(topic_crosswalk_detected, 1, CB_CrosswalkDetected);
  ros::Subscriber sub_rotary =
      nh.subscribe(topic_rotary_detected, 1, CB_RotaryDetected);
  ros::Subscriber sub_parking =
      nh.subscribe(topic_parking_detected, 1, CB_ParkingDetected);
  ros::Subscriber sub_parking_lock =
      nh.subscribe(topic_parking_bag_lock, 1, CB_ParkingBagLock);
  ros::Subscriber sub_ab_left =
      nh.subscribe(topic_ab_left, 1, CB_AB_Left);
  ros::Subscriber sub_ab_right =
      nh.subscribe(topic_ab_right, 1, CB_AB_Right);
  ros::Publisher pub_ab_enable =
      nh.advertise<std_msgs::Bool>(topic_ab_enable, 1, true);

  // ===== 각 미션 초기화 =====
  mission_lane_init(nh, pnh);
  mission_labacorn_init(nh, pnh);
  mission_gate_init(nh, pnh);
  mission_crosswalk_init(nh, pnh);
  mission_rotary_init(nh, pnh);
  mission_parking_init(nh, pnh);
  mission_ab_left_init(nh, pnh);
  mission_ab_right_init(nh, pnh);

  ROS_INFO("[main_node] all mission init done");

  ros::Rate rate(10.0);  // 10 Hz

  MissionState prev_state = g_current_state;

  while (ros::ok())
  {
    ros::spinOnce();

    // -----------------------------
    // 1) 미션 상태 결정 로직
    //    (우선순위: 게이트 > 횡단보도 > 라바콘 > 기본 차선)
    // -----------------------------
    if (g_parking_bag_lock)
    {
      g_current_state = MISSION_PARKING;
    }
    else if (g_gate_detected)
    {
      g_current_state = MISSION_GATE;
    }
    else if (g_crosswalk_detected)
    {
      g_current_state = MISSION_CROSSWALK;
    }
    else if (g_ab_action_running || g_ab_latched_dir != -1)
    {
      g_current_state = MISSION_AB;
    }
    else if (g_labacorn_detected)
    {
      g_current_state = MISSION_LABACORN;
    }
    else if (g_rotary_detected)
    {
      g_current_state = MISSION_ROTARY;
    }
    else if (g_parking_detected)
    {
      g_current_state = MISSION_PARKING;
    }
    else
    {
      g_current_state = MISSION_LANE;
    }

    // 상태 변경 시 로그
    if (g_current_state != prev_state)
    {
      const char* state_name = "LANE";
      if (g_current_state == MISSION_LABACORN)      state_name = "LABACORN";
      else if (g_current_state == MISSION_GATE)     state_name = "GATE";
      else if (g_current_state == MISSION_CROSSWALK) state_name = "CROSSWALK";
      else if (g_current_state == MISSION_ROTARY) state_name = "ROTARY";
      else if (g_current_state == MISSION_PARKING) state_name = "PARKING";
      else if (g_current_state == MISSION_AB) state_name = "AB";

      ROS_INFO("[main_node] Mission changed -> %s", state_name);

      // 1. 라바콘 1회 -> AB ENABLE TOPIC --> /root/autorace_kkk_ws/src/perception_node/src/ab_sign/traffic_sign.py 구독
      // 2. /root/autorace_kkk_ws/src/perception_node/src/ab_sign/traffic_sign.py가 left, right 를 뽑는다.
      // 3. main이 다시 구독 -->   /root/autorace_kkk_ws/src/decision_node/src/mission_ab/mission_ab_left.cpp, 또는 right를 실행하도록 한다.
      if (g_current_state == MISSION_LABACORN)
      {
        g_labacorn_count++;
        if (g_labacorn_count == 1)
        {
          std_msgs::Bool enable_msg;
          enable_msg.data = true; // 1 -> TRUE,
          pub_ab_enable.publish(enable_msg);
          ROS_INFO("[main_node] LABACORN first entry -> enabling AB perception for 8s");
        }
      }
      else if (g_current_state == MISSION_AB)
      {
        if (g_ab_latched_dir == -1)
        {
          if (g_ab_left_detected) g_ab_latched_dir = 0; // left detect -> left
          else if (g_ab_right_detected) g_ab_latched_dir = 1; // right detect -> right 
        }
        if (g_ab_latched_dir == 0) // left
        {
          mission_ab_left_reset();
          g_ab_action_running = true;
        }
        else if (g_ab_latched_dir == 1)
        {
          mission_ab_right_reset();
          g_ab_action_running = true;
        }
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
        mission_lane_step();
        break;

      case MISSION_CROSSWALK:
        mission_crosswalk_step();   // ← 여기 안에서 7초 정지 + 2초 직진
        break;

      case MISSION_LABACORN:
        // 라바콘, 터널 둘 다 이 로직 사용
        mission_labacorn_step();
        break;

      case MISSION_GATE:
        mission_gate_step();
        break;
      
      case MISSION_ROTARY:
        mission_rotary_step();
        break;

      case MISSION_PARKING:
        mission_parking_step();
        break;

      default:
        mission_lane_step();
        break;
    }

    rate.sleep();
  }

  return 0;
}
