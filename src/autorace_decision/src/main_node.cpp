// main_node.cpp
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <string>
#include <vector>

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

// mission_crosswalk_toMain.cpp
void mission_crosswalk_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_crosswalk_step();

// mission_rotary.cpp
void mission_rotary_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_rotary_step();

// mission_AB_left_toMain.cpp (왼쪽 차선 8초 추종)
void mission_AB_left_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_AB_left_step();
bool mission_AB_left_is_done();

// mission_color_toMain.cpp (빨강/파랑 구간 속도 변경 미션)
void mission_color_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
// color_code: 0=none, 1=red, 2=blue
void mission_color_step(int color_code);

// -------------------- 미션 상태 enum --------------------
enum MissionState
{
  MISSION_LANE = 0,
  MISSION_LABACORN,
  MISSION_GATE,
  MISSION_CROSSWALK,
  MISSION_ROTARY,
  MISSION_COLOR       // 빨강/파랑 속도 제어
};

MissionState g_current_state = MISSION_LANE;
std::vector<MissionState> g_mission_plan = {
    MISSION_COLOR,
    MISSION_CROSSWALK,
    MISSION_LABACORN,
    MISSION_ROTARY,
    MISSION_LABACORN,
    MISSION_LABACORN,
    MISSION_GATE
};
size_t g_stage_index = 0;
bool   g_stage_active = false;
ros::Time g_stage_start;
ros::Time g_last_log_time;

// -------------------- 감지 토픽 상태 플래그 --------------------
bool g_labacorn_detected   = false;
bool g_gate_detected       = false;
bool g_crosswalk_detected  = false;
bool g_rotary_detected     = false;

// 빨강/파랑 차선 색
int  g_lane_color_code     = 0;   // 0=none, 1=red, 2=blue

// AB 하드코딩 회전용 타이머
double g_first_lane_duration = 3.0;  // 라바콘 후 차선 주행 시간
double g_first_ab_duration   = 8.0;  // 라바콘 후 AB 회전 시간 (하드코딩)
int    g_first_ab_dir        = -1;   // -1=left (오른쪽 주석처리) <- 대회때 표지판 보고 바로 변경하고 실행하기,,
bool   g_first_sequence_running = false;
bool   g_first_sequence_done    = false;
ros::Time g_first_sequence_start;
int    g_labacorn_pass_count = 0;

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

// -------------------- 콜백: Rotary 감지 --------------------
void CB_RotaryDetected(const std_msgs::Bool::ConstPtr& msg)
{
  g_rotary_detected = msg->data;
}

// -------------------- 콜백: lane_color_node (center_color_px) --------------------
void CB_LaneColor(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  int code = static_cast<int>(msg->point.x);
  if (code < 0) code = 0;
  if (code > 2) code = 2;
  g_lane_color_code = code;
}

// -------------------- 현재 미션 단계 완료 조건 --------------------
bool isDetectedForState(MissionState s)
{
  switch (s)
  {
    case MISSION_COLOR:     return (g_lane_color_code > 0);
    case MISSION_CROSSWALK: return g_crosswalk_detected;
    case MISSION_LABACORN:  return g_labacorn_detected;
    case MISSION_ROTARY:    return g_rotary_detected;
    case MISSION_GATE:      return g_gate_detected;
    default:                return false;
  }
}

const char* missionName(MissionState s)
{
  switch (s)
  {
    case MISSION_LANE:      return "MISSION_LANE";
    case MISSION_LABACORN:  return "MISSION_LABACORN";
    case MISSION_GATE:      return "MISSION_GATE";
    case MISSION_CROSSWALK: return "MISSION_CROSSWALK";
    case MISSION_ROTARY:    return "MISSION_ROTARY";
    case MISSION_COLOR:     return "MISSION_COLOR";
    default:                return "MISSION_UNKNOWN";
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
  std::string topic_center_color;   // lane_color_node 출력

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

  // lane_color_node 토픽
  pnh.param<std::string>("center_color_topic",
                         topic_center_color,
                         std::string("/perception/center_color_px"));

  ROS_INFO("[main_node] subscribe "
           "labacorn='%s', gate='%s', crosswalk='%s', rotary='%s', "
           "center_color='%s'",
           ros::names::resolve(topic_labacorn_detected).c_str(),
           ros::names::resolve(topic_gate_detected).c_str(),
           ros::names::resolve(topic_crosswalk_detected).c_str(),
           ros::names::resolve(topic_rotary_detected).c_str(),
           ros::names::resolve(topic_center_color).c_str());

  // ===== 감지 토픽 구독 =====
  ros::Subscriber sub_labacorn =
      nh.subscribe(topic_labacorn_detected, 1, CB_LabacornDetected);
  ros::Subscriber sub_gate =
      nh.subscribe(topic_gate_detected, 1, CB_GateDetected);
  ros::Subscriber sub_crosswalk =
      nh.subscribe(topic_crosswalk_detected, 1, CB_CrosswalkDetected);
  ros::Subscriber sub_rotary =
      nh.subscribe(topic_rotary_detected, 1, CB_RotaryDetected);

  ros::Subscriber sub_center_color =
      nh.subscribe(topic_center_color, 1, CB_LaneColor);

  // ===== 각 미션 초기화 =====
  mission_lane_init(nh, pnh);
  mission_labacorn_init(nh, pnh);
  mission_gate_init(nh, pnh);
  mission_crosswalk_init(nh, pnh);
  mission_rotary_init(nh, pnh);
  mission_AB_left_init(nh, pnh);
  mission_color_init(nh, pnh);

  ROS_INFO("[main_node] all mission init done");

  ros::Rate rate(30.0);  // 30 Hz

  MissionState prev_state = g_current_state;

  while (ros::ok())
  {
    ros::spinOnce();

    // -----------------------------
    // 1) 미션 상태 결정 로직 (주어진 순서대로 진행)
    MissionState target_state = g_mission_plan[g_stage_index];

    bool detected = isDetectedForState(target_state);

    // 아직 단계 진입 전: 감지되면 시작
      if (!g_stage_active) {
        if (detected) {
          g_stage_active = true;
          g_stage_start  = ros::Time::now();
          g_current_state = target_state;
          if (target_state == MISSION_LABACORN) {
            ++g_labacorn_pass_count;
            g_first_sequence_running = false;
            g_first_sequence_done = false;
          }
          ROS_INFO("[main_node] stage %zu START (state=%d)", g_stage_index, g_current_state);
        } else {
          g_current_state = MISSION_LANE;  // 기본은 차선 추종
        }
      }
    // 단계 진행 중
    else {
      g_current_state = target_state;

      // 완료 조건: 감지 해제 + 최소 지속 시간(안전상 여유)
      double elapsed = (ros::Time::now() - g_stage_start).toSec();
      double min_dwell = 0.5;  // 기본 최소 지속 시간
      if (target_state == MISSION_CROSSWALK) min_dwell = 3.0;  // 정지/직진 구간 확보

      if (!detected && elapsed >= min_dwell) {
        g_stage_active = false;
        if (g_stage_index + 1 < g_mission_plan.size()) {
          ++g_stage_index;
          ROS_INFO("[main_node] stage COMPLETE -> advance to %zu", g_stage_index);
        } else {
          ROS_INFO("[main_node] final stage complete, stay on last mission");
        }
      }
    }

    if (prev_state != g_current_state) {
      ROS_INFO("[main_node] state change: %d -> %d", prev_state, g_current_state);
      prev_state = g_current_state;
    }

    // -----------------------------
    // 2) 미션 실행
    // -----------------------------
    switch (g_current_state)
    {
      case MISSION_LANE:
        mission_lane_step();
        break;

      case MISSION_LABACORN:
        if (g_labacorn_pass_count == 1 && !g_first_sequence_done) {
          // 첫 라바콘: 차선 3초 -> AB 하드코딩 8초
          if (!g_first_sequence_running) {
            g_first_sequence_running = true;
            g_first_sequence_start = ros::Time::now();
          }
          double seq_elapsed = (ros::Time::now() - g_first_sequence_start).toSec();

          if (seq_elapsed < g_first_lane_duration) {
            mission_lane_step();
          } else {
            mission_AB_left_step();
            if (mission_AB_left_is_done()) {
              g_first_sequence_done = true;
              g_first_sequence_running = false;
              g_stage_active = false;
              if (g_stage_index + 1 < g_mission_plan.size()) {
                ++g_stage_index;
                ROS_INFO("[main_node] first labacorn seq done -> advance to %zu", g_stage_index);
              } else {
                ROS_INFO("[main_node] final stage complete, stay on last mission");
              }
              g_current_state = MISSION_LANE;
            }
          }
        } else {
          mission_labacorn_step();
        }
        break;

      case MISSION_GATE:
        mission_gate_step();
        break;

      case MISSION_CROSSWALK:
        mission_crosswalk_step();
        break;

      case MISSION_ROTARY:
        mission_rotary_step();
        break;

      case MISSION_COLOR:
        mission_color_step(g_lane_color_code);
        break;
    }

    // -----------------------------
    // 3) 현재 미션 주기적 로그 (1초 간격)
    // -----------------------------
    ros::Time now_log = ros::Time::now();
    if (g_last_log_time.isZero() || (now_log - g_last_log_time).toSec() >= 1.0) {
      g_last_log_time = now_log;
      ROS_INFO("[main_node] current mission: %s (stage %zu/%zu)",
               missionName(g_current_state),
               g_stage_index + 1,
               g_mission_plan.size());
    }

    rate.sleep();
  }

}
