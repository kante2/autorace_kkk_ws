#include <ros/ros.h>
#include <std_msgs/Bool.h>

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

// -------------------- 미션 상태 enum --------------------
enum MissionState
{
  MISSION_LANE = 0,
  MISSION_LABACORN,
  MISSION_GATE
};

MissionState g_current_state = MISSION_LANE;

// 감지 토픽 상태
bool g_labacorn_detected = false;
bool g_gate_detected     = false;

// (필요하면 나중에 “이 미션은 이미 초기화 했는지” 플래그도 둘 수 있음)
// 지금은 3개 미션을 모두 시작할 때 init() 한 번씩 호출한다고 가정

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

  pnh.param<std::string>("labacorn_detected_topic",
                         topic_labacorn_detected,
                         std::string("/labacorn_detected"));
  pnh.param<std::string>("gate_detected_topic",
                         topic_gate_detected,
                         std::string("/gate_detected"));

  ROS_INFO("[main_node] subscribe labacorn='%s', gate='%s'",
           ros::names::resolve(topic_labacorn_detected).c_str(),
           ros::names::resolve(topic_gate_detected).c_str());

  // ===== 감지 토픽 구독 =====
  ros::Subscriber sub_labacorn =
      nh.subscribe(topic_labacorn_detected, 1, CB_LabacornDetected);
  ros::Subscriber sub_gate =
      nh.subscribe(topic_gate_detected, 1, CB_GateDetected);

  // ===== 각 미션 초기화 =====
  // lane / labacorn / gate 각각 자신의 토픽, 파라미터, pub/sub 세팅을 init 안에서 수행
  mission_lane_init(nh, pnh);
  mission_labacorn_init(nh, pnh);
  mission_gate_init(nh, pnh);

  ROS_INFO("[main_node] all mission init done");

  ros::Rate rate(30.0);  // 30 Hz

  MissionState prev_state = g_current_state;

  while (ros::ok())
  {
    ros::spinOnce();

    // -----------------------------
    // 1) 미션 상태 결정 로직
    // -----------------------------
    // 우선순위 예시:
    //   - gate_detected == true    → GATE 미션
    //   - labacorn_detected == true→ LABACORN 미션
    //   - 둘 다 false              → 기본 LANE 미션
    if (g_gate_detected)
    {
      g_current_state = MISSION_GATE;
    }
    else if (g_labacorn_detected)
    {
      g_current_state = MISSION_LABACORN;
    }
    else
    {
      g_current_state = MISSION_LANE;
    }

    // 상태 변경 시 로그
    if (g_current_state != prev_state)
    {
      const char* state_name = "LANE";
      if (g_current_state == MISSION_LABACORN) state_name = "LABACORN";
      else if (g_current_state == MISSION_GATE) state_name = "GATE";

      ROS_INFO("[main_node] Mission changed -> %s", state_name);
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

      case MISSION_LABACORN:
        mission_labacorn_step();
        break;

      case MISSION_GATE:
        mission_gate_step();
        break;

      default:
        mission_lane_step();
        break;
    }

    rate.sleep();
  }

  return 0;
}
