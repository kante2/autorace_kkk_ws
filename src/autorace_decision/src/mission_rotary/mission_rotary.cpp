// mission_rotary.cpp
// rotary_main.cpp의 라이다 전처리/클러스터링/감지/감속 로직을 함수화하여
// main_node와 디버그용 main에서 재사용

#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include "clustering_visualization.hpp"
#include "lidar_preprocessing_rotary.cpp"
#include "object_detect.hpp"

namespace {
// 토픽 이름
std::string g_topic_scan;
std::string g_topic_rotary_detected;
std::string g_topic_marker;
std::string g_topic_motor;
std::string g_topic_servo;

// DBSCAN 파라미터
double g_eps = 0.3;
int g_min_pts = 7;

// 속도/모터 파라미터
double g_cruise_speed_mps = 1.0;   // 감지 없을 때 기본 속도
double g_slow_speed_mps = 0.5;     // 1m 이내 감지 시 목표 속도
double g_speed_step_mps = 0.1;     // 한 스텝당 변화량(감속/가속)
double g_motor_min_cmd = 0.0;
double g_motor_max_cmd = 2000.0;
double g_motor_gain = 1500.0;      // m/s -> 모터 명령

// 서보 파라미터(중립 유지)
double g_servo_center = 0.5;
double g_servo_min = 0.0;
double g_servo_max = 1.0;

// 상태
ros::Publisher g_cluster_pub;
ros::Publisher g_rotary_detected_pub;
ros::Publisher g_motor_pub;
ros::Publisher g_servo_pub;
ros::Subscriber g_scan_sub;
ros::Timer g_detection_timer;
sensor_msgs::LaserScan::ConstPtr g_scan_msg;

double g_current_speed_mps = 0.0;

inline double clamp(double x, double lo, double hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- 콜백 --------------------
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) { g_scan_msg = scan_msg; }

// -------------------- 내부 처리 --------------------
// publish_control=false: 감지 토픽만 내고 모터/서보는 내지 않는다(미션 비활성화 시에도 감지는 계속).
void processOnce(bool publish_control) {
  if (!g_scan_msg) {
    return;
  }

  const LidarProcessingResult processed = preprocessLidar(*g_scan_msg);
  const DetectionResult detection = detect(processed.angle_ranges_deg, processed.dist_ranges, g_eps, g_min_pts);

  bool has_close_object = false;
  if (detection.detected) {
    for (std::size_t cluster_id = 0; cluster_id < detection.centroids.size(); ++cluster_id) {
      const ClusterCentroid &center = detection.centroids[cluster_id];
      ROS_INFO("id=%zu angle(deg)=%.3f distance(m)=%.3f", cluster_id, center.angle, center.distance);
      if (center.distance <= 1.0) {
        has_close_object = true;
      }
    }
    publishClusterMarkers(g_cluster_pub, detection.centroids);
  } else {
    ROS_INFO("not detected");
  }

  // 감지 여부 퍼블리시 (/rotary_detected 등)
  std_msgs::Bool detected_msg;
  detected_msg.data = has_close_object;
  g_rotary_detected_pub.publish(detected_msg);

  if (!publish_control) {
    return;
  }

  // 감속/가속 속도 결정
  const double target_speed = has_close_object ? g_slow_speed_mps : g_cruise_speed_mps;
  if (g_current_speed_mps > target_speed) {
    g_current_speed_mps = std::max(target_speed, g_current_speed_mps - g_speed_step_mps);
  } else if (g_current_speed_mps < target_speed) {
    g_current_speed_mps = std::min(target_speed, g_current_speed_mps + g_speed_step_mps);
  }

  // 모터/서보 퍼블리시 (서보는 중립)
  double motor_cmd = clamp(g_motor_gain * g_current_speed_mps, g_motor_min_cmd, g_motor_max_cmd);
  double servo_cmd = clamp(g_servo_center, g_servo_min, g_servo_max);

  std_msgs::Float64 motor_msg;
  std_msgs::Float64 servo_msg;
  motor_msg.data = motor_cmd;
  servo_msg.data = servo_cmd;

  g_motor_pub.publish(motor_msg);
  g_servo_pub.publish(servo_msg);

  ROS_INFO_THROTTLE(0.5,
                    "[mission_rotary] detected=%d target_v=%.2f cur_v=%.2f motor=%.1f servo=%.2f",
                    static_cast<int>(has_close_object), target_speed, g_current_speed_mps, motor_cmd,
                    servo_cmd);
}

void detectionTimerCb(const ros::TimerEvent &) {
  // 미션 활성 여부와 무관하게 감지 상태를 주기적으로 퍼블리시
  processOnce(false);
}
}  // namespace

// =====================================================
// main_node.cpp 에서 사용할 init / step 함수
// =====================================================

void mission_rotary_init(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  ROS_INFO("[mission_rotary] init");

  // 파라미터 로드
  pnh.param<std::string>("scan_topic", g_topic_scan, std::string("/scan"));
  pnh.param<std::string>("rotary_detected_topic", g_topic_rotary_detected, std::string("/rotary_detected"));
  pnh.param<std::string>("marker_topic", g_topic_marker, std::string("rotary/obstacle_markers"));
  pnh.param<std::string>("motor_topic", g_topic_motor, std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic", g_topic_servo, std::string("/commands/servo/position"));

  pnh.param<double>("eps", g_eps, 0.3);
  pnh.param<int>("min_pts", g_min_pts, 7);

  pnh.param<double>("cruise_speed_mps", g_cruise_speed_mps, 1.0);
  pnh.param<double>("slow_speed_mps", g_slow_speed_mps, 0.5);
  pnh.param<double>("speed_step_mps", g_speed_step_mps, 0.1);

  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);
  pnh.param<double>("motor_max_cmd", g_motor_max_cmd, 2000.0);
  pnh.param<double>("motor_gain", g_motor_gain, 1500.0);

  pnh.param<double>("servo_center", g_servo_center, 0.5);
  pnh.param<double>("servo_min", g_servo_min, 0.0);
  pnh.param<double>("servo_max", g_servo_max, 1.0);

  ROS_INFO("[mission_rotary] subscribe scan='%s'",
           ros::names::resolve(g_topic_scan).c_str());
  ROS_INFO("[mission_rotary] publish detected='%s', markers='%s', motor='%s', servo='%s'",
           ros::names::resolve(g_topic_rotary_detected).c_str(),
           ros::names::resolve(g_topic_marker).c_str(),
           ros::names::resolve(g_topic_motor).c_str(),
           ros::names::resolve(g_topic_servo).c_str());

  // Pub/Sub 설정
  g_cluster_pub = initClusterVisualizer(nh, g_topic_marker);
  g_rotary_detected_pub = nh.advertise<std_msgs::Bool>(g_topic_rotary_detected, 1);
  g_motor_pub = nh.advertise<std_msgs::Float64>(g_topic_motor, 10);
  g_servo_pub = nh.advertise<std_msgs::Float64>(g_topic_servo, 10);
  g_scan_sub = nh.subscribe(g_topic_scan, 1, scanCallback);
  g_detection_timer = nh.createTimer(ros::Duration(1.0 / 30.0), detectionTimerCb);

  g_scan_msg.reset();
  g_current_speed_mps = g_cruise_speed_mps;

  ROS_INFO("[mission_rotary] init done");
}

// main loop에서 호출
void mission_rotary_step() {
  processOnce(true);
}
