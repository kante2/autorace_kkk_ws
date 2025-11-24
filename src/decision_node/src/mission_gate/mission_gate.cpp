// mission 7: 차단기 미션
// 차단기 앞에서 정지 후 차단기가 올라가면 주행하는 미션

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

// ===== 전역 ROS I/O =====
static ros::Subscriber g_sub_gate_detected;
static ros::Subscriber g_sub_gate_points;
static ros::Publisher g_pub_speed;

// 토픽 이름
static std::string g_motor_topic;
static std::string g_gate_detected_topic;
static std::string g_gate_points_topic;

// ===== 전역 파라미터 =====
static double g_basic_speed = 1000.0;   // 게이트 올라가 있을 때 기본 속도 (모터 cmd)
static double g_stop_speed = 0.0;       // 정지 속도 (모터 cmd)
static double g_gate_stop_dist = 2.0;   // 로봇-게이트 중심 거리 임계값 [m]

// 게이트 토픽 타임아웃
static double g_gate_timeout_sec = 0.5;
static bool g_have_gate_detect = false;
static bool g_have_gate_dist = false;
static ros::Time g_last_gate_time;

// 게이트 상태 / 거리
static bool g_gate_down_final = false;
static double g_gate_dist = std::numeric_limits<double>::infinity();
static bool g_have_gate_state = false;

// -------------------- 입력 콜백 --------------------
static void gateDetectedCB(const std_msgs::Bool::ConstPtr& msg)
{
  g_gate_down_final = msg->data;
  g_have_gate_detect = true;
  g_last_gate_time = ros::Time::now();
}

static void gatePointsCB(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  g_gate_dist = std::numeric_limits<double>::infinity();
  g_have_gate_dist = false;

  // PointCloud2에서 평균 거리 계산 (gate_node가 게이트 클러스터만 퍼블리시)
  if (msg->width == 0 || msg->height == 0) {
    g_last_gate_time = ros::Time::now();
    return;
  }

  double sum_dist = 0.0;
  std::size_t count = 0;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    const double x = static_cast<double>(*iter_x);
    const double y = static_cast<double>(*iter_y);
    if (!std::isfinite(x) || !std::isfinite(y)) continue;
    const double dist = std::sqrt(x * x + y * y);
    if (!std::isfinite(dist)) continue;
    sum_dist += dist;
    ++count;
  }

  if (count > 0) {
    g_gate_dist = sum_dist / static_cast<double>(count);
    g_have_gate_dist = true;
  }

  g_last_gate_time = ros::Time::now();
}

// 속도 제어용 publish 함수
static void pubGateSign(bool stop)
{
  std_msgs::Float64 speed_msg;

  if (stop)
  {
    speed_msg.data = g_stop_speed;
    ROS_INFO_THROTTLE(1.0, "GATE DOWN & CLOSE: Stopping");
  }
  else
  {
    speed_msg.data = g_basic_speed;
    ROS_INFO_THROTTLE(1.0, "GATE UP or FAR: Moving forward");
  }

  g_pub_speed.publish(speed_msg);
}

// stop 여부 계산 (publish는 하지 않음)
static bool SpeedControl(bool gate_down_final, double gate_dist)
{
  bool should_stop = false;

  if (gate_down_final && std::isfinite(gate_dist) && gate_dist <= g_gate_stop_dist)
  {
    should_stop = true;
  }

  ROS_INFO_THROTTLE(0.5,
                    "[mission_gate] gate_down_final=%d, dist=%.3f, stop_dist=%.3f -> %s",
                    gate_down_final ? 1 : 0,
                    gate_dist,
                    g_gate_stop_dist,
                    should_stop ? "STOP" : "GO");

  return should_stop;
}

// =====================================================
// main_node.cpp 에서 사용할 init / step 함수
// =====================================================

void mission_gate_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[mission_gate] mission_gate_init()");

  pnh.param("basic_speed", g_basic_speed, g_basic_speed);
  pnh.param("stop_speed", g_stop_speed, g_stop_speed);
  pnh.param("gate_stop_dist", g_gate_stop_dist, g_gate_stop_dist);
  pnh.param("gate_timeout_sec", g_gate_timeout_sec, g_gate_timeout_sec);

  pnh.param<std::string>("motor_topic", g_motor_topic, std::string("/commands/motor/speed"));
  pnh.param<std::string>("gate_detected_topic", g_gate_detected_topic, std::string("/gate_detected"));
  pnh.param<std::string>("gate_points_topic", g_gate_points_topic, std::string("/gate_points"));

  ROS_INFO("[mission_gate] subscribe gate_detected='%s', gate_points='%s'",
           ros::names::resolve(g_gate_detected_topic).c_str(),
           ros::names::resolve(g_gate_points_topic).c_str());
  ROS_INFO("[mission_gate] publish motor='%s'",
           ros::names::resolve(g_motor_topic).c_str());

  g_pub_speed = nh.advertise<std_msgs::Float64>(g_motor_topic, 1);
  g_sub_gate_detected = nh.subscribe(g_gate_detected_topic, 1, gateDetectedCB);
  g_sub_gate_points = nh.subscribe(g_gate_points_topic, 1, gatePointsCB);

  g_gate_down_final = false;
  g_gate_dist = std::numeric_limits<double>::infinity();
  g_have_gate_state = false;
  g_have_gate_detect = false;
  g_have_gate_dist = false;

  ROS_INFO("[mission_gate] mission_gate_init done");
}

// main_node.cpp 의 while 루프 안에서 주기적으로 호출
void mission_gate_step()
{
  ros::Time now = ros::Time::now();

  bool fresh_gate = false;
  if (g_have_gate_detect || g_have_gate_dist)
  {
    double dt = (now - g_last_gate_time).toSec();
    fresh_gate = (dt <= g_gate_timeout_sec);
  }

  bool should_stop = true;  // 기본은 STOP

  if (fresh_gate && (g_have_gate_detect || g_have_gate_dist))
  {
    should_stop = SpeedControl(g_gate_down_final, g_gate_dist);
  }
  else
  {
    ROS_INFO_THROTTLE(1.0, "[mission_gate] waiting gate topics... (timeout) -> STOP");
    should_stop = true;
  }

  if (should_stop)
  {
    // 게이트 내려감 + 근접/타임아웃 시 정지 신호만 내려보냄
    pubGateSign(true);
  }
  else
  {
    // 게이트가 올라갔거나 기준거리 밖이면 속도 명령은 내지 않고
    // main_node가 기본 미션(차선)으로 넘어가도록 맡긴다.
  }
}
