// mission 7: 차단기 미션
// 차단기 앞에서 정지 후 차단기가 올라가면 주행하는 미션

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <laser_geometry/laser_geometry.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

enum GateState
{
  GATE_UNKNOWN = 0,
  GATE_DOWN,
  GATE_UP
};

struct Point2D
{
  double x;
  double y;
};

#include "lidar_preprocessing_gate.cpp"
#include "gate_detect.cpp"

// ===== 전역 ROS I/O =====
static ros::Subscriber g_sub_scan;
static ros::Publisher g_pub_gate_detected;
static ros::Publisher g_pub_speed;
static ros::Publisher g_pub_cloud;
static ros::Publisher g_pub_marker;  // marker는 현재 사용 안 하지만 인터페이스 유지
static laser_geometry::LaserProjection g_projector;
static ros::Timer g_detect_timer;

// 토픽 이름
static std::string g_scan_topic;
static std::string g_motor_topic;
static std::string g_cloud_topic;
static std::string g_marker_topic;
static std::string g_gate_detected_topic;

// ===== 전역 파라미터 =====
static double g_eps = 0.2;              // 클러스터 간 거리 [m]
static int g_min_samples = 10;          // DBSCAN 최소 점 수
static double g_range_min = 0.10;       // 전처리 range 최소 [m]
static double g_range_max = 0.50;       // 전처리 range 최대 [m]
static double g_front_min_deg = 60.0;   // ROI 시작 각도
static double g_front_max_deg = 300.0;  // ROI 끝 각도
static double g_basic_speed = 1000.0;   // 게이트 올라가 있을 때 기본 속도 (모터 cmd)
static double g_stop_speed = 0.0;       // 정지 속도 (모터 cmd)
static double g_gate_stop_dist = 2.0;   // 로봇-게이트 중심 거리 임계값 [m]

// 게이트 상태 히스테리시스
static int g_gate_down_count = 0;
static int g_gate_up_count = 0;
static int g_gate_down_thresh = 3;
static int g_gate_up_thresh = 3;

// 스캔 타임아웃
static double g_scan_timeout_sec = 0.5;
static bool g_have_scan_time = false;
static ros::Time g_last_scan_time;

// 게이트 상태 / 거리
static bool g_gate_down_final = false;
static double g_gate_dist = std::numeric_limits<double>::infinity();
static bool g_have_gate_state = false;

// -------------------- 감지 토픽 퍼블리시 --------------------
static void publishGateDetected(const ros::TimerEvent&)
{
  ros::Time now = ros::Time::now();
  bool have_scan = false;
  if (g_have_scan_time)
  {
    double dt = (now - g_last_scan_time).toSec();
    have_scan = (dt <= g_scan_timeout_sec);
  }

  bool detected = have_scan && g_have_gate_state && g_gate_down_final;

  std_msgs::Bool msg;
  msg.data = detected;
  g_pub_gate_detected.publish(msg);
}

// 속도 제어용 publish 함수
static void pubGateSign(bool stop)
{
  std_msgs::Float64 speed_msg;

  if (stop)
  {
    speed_msg.data = g_stop_speed;
    ROS_INFO_THROTTLE(1.0, "→ GATE DOWN & CLOSE: Stopping");
  }
  else
  {
    speed_msg.data = g_basic_speed;
    ROS_INFO_THROTTLE(1.0, "→ GATE UP or FAR: Moving forward");
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

// ================== Scan Callback ==================
static void scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  g_last_scan_time = ros::Time::now();
  g_have_scan_time = true;

  const bool make_cloud = (g_pub_cloud.getNumSubscribers() > 0);
  const GateLidarPreprocessResult preprocessed =
      preprocessGateLidar(*scan, g_range_min, g_range_max, g_front_min_deg, g_front_max_deg, g_projector, make_cloud);

  if (preprocessed.has_cloud && make_cloud)
  {
    g_pub_cloud.publish(preprocessed.cloud);
  }

  GateDetectionResult detection =
      detectGate(preprocessed.points,
                 g_eps,
                 g_min_samples,
                 g_gate_down_thresh,
                 g_gate_up_thresh,
                 g_gate_down_count,
                 g_gate_up_count);

  if (detection.has_gate)
  {
    ROS_INFO_THROTTLE(0.5,
                      "[mission_gate] center=(%.3f, %.3f), dist=%.3f, clusters=%d",
                      detection.gate_cx,
                      detection.gate_cy,
                      detection.gate_dist,
                      detection.cluster_count);
  }
  else
  {
    ROS_INFO_THROTTLE(0.5, "[mission_gate] no valid gate cluster -> assume UP");
  }

  g_gate_down_final = (detection.gate_state == GATE_DOWN);
  g_gate_dist = detection.gate_dist;
  g_have_gate_state = true;
}

// =====================================================
// main_node.cpp 에서 사용할 init / step 함수
// =====================================================

void mission_gate_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[mission_gate] mission_gate_init()");

  pnh.param("eps", g_eps, g_eps);
  pnh.param("min_samples", g_min_samples, g_min_samples);
  pnh.param("range_min", g_range_min, g_range_min);
  pnh.param("range_max", g_range_max, g_range_max);
  pnh.param("front_min_deg", g_front_min_deg, g_front_min_deg);
  pnh.param("front_max_deg", g_front_max_deg, g_front_max_deg);
  pnh.param("basic_speed", g_basic_speed, g_basic_speed);
  pnh.param("stop_speed", g_stop_speed, g_stop_speed);
  pnh.param("gate_down_thresh", g_gate_down_thresh, g_gate_down_thresh);
  pnh.param("gate_up_thresh", g_gate_up_thresh, g_gate_up_thresh);
  pnh.param("gate_stop_dist", g_gate_stop_dist, g_gate_stop_dist);
  pnh.param("scan_timeout_sec", g_scan_timeout_sec, g_scan_timeout_sec);

  pnh.param<std::string>("scan_topic", g_scan_topic, std::string("/scan"));
  pnh.param<std::string>("motor_topic", g_motor_topic, std::string("/commands/motor/speed"));
  pnh.param<std::string>("cloud_topic", g_cloud_topic, std::string("/scan_points"));
  pnh.param<std::string>("marker_topic", g_marker_topic, std::string("/gate_cluster_marker"));
  pnh.param<std::string>("gate_detected_topic", g_gate_detected_topic, std::string("/gate_detected"));

  ROS_INFO("[mission_gate] subscribe scan='%s'", ros::names::resolve(g_scan_topic).c_str());
  ROS_INFO("[mission_gate] publish motor='%s', cloud='%s', marker='%s', gate_detected='%s'",
           ros::names::resolve(g_motor_topic).c_str(),
           ros::names::resolve(g_cloud_topic).c_str(),
           ros::names::resolve(g_marker_topic).c_str(),
           ros::names::resolve(g_gate_detected_topic).c_str());

  g_sub_scan = nh.subscribe<sensor_msgs::LaserScan>(g_scan_topic, 1, scanCallback);
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(g_cloud_topic, 1);
  g_pub_speed = nh.advertise<std_msgs::Float64>(g_motor_topic, 1);
  g_pub_marker = nh.advertise<visualization_msgs::Marker>(g_marker_topic, 1);
  g_pub_gate_detected = nh.advertise<std_msgs::Bool>(g_gate_detected_topic, 1);
  g_detect_timer = nh.createTimer(ros::Duration(1.0 / 30.0), publishGateDetected);

  g_gate_down_count = 0;
  g_gate_up_count = 0;
  g_gate_down_final = false;
  g_gate_dist = std::numeric_limits<double>::infinity();
  g_have_gate_state = false;
  g_have_scan_time = false;

  ROS_INFO("[mission_gate] mission_gate_init done");
}

// main_node.cpp 의 while 루프 안에서 주기적으로 호출
void mission_gate_step()
{
  ros::Time now = ros::Time::now();

  bool have_scan = false;
  if (g_have_scan_time)
  {
    double dt = (now - g_last_scan_time).toSec();
    have_scan = (dt <= g_scan_timeout_sec);
  }

  bool should_stop = true;  // 기본은 STOP

  if (have_scan && g_have_gate_state)
  {
    should_stop = SpeedControl(g_gate_down_final, g_gate_dist);
  }
  else
  {
    if (!have_scan)
    {
      ROS_INFO_THROTTLE(1.0, "[mission_gate] waiting LaserScan... (timeout) -> STOP");
    }
    else
    {
      ROS_INFO_THROTTLE(1.0, "[mission_gate] have_scan but no gate_state -> STOP");
    }
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
