// mission_rotary.cpp
#include <string>
#include <algorithm>
#include <limits>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Polygon.h>

namespace {
// 토픽 이름
std::string g_topic_rotary_detected;
std::string g_topic_motor;
std::string g_topic_centroids;
std::string g_topic_right_centers;
std::string g_topic_servo;

// 속도/모터 파라미터
double g_cruise_speed_mps = 1.0;   // 감지 없을 때 기본 속도
double g_stop_speed_mps   = 0.0;   // 감지 시 정지
double g_motor_min_cmd = 0.0;
double g_motor_max_cmd = 2000.0;
double g_motor_gain = 3846.15;     // m/s -> 모터 명령 (1000cmd = 0.26m/s -> 1/0.00026)
double g_far_motor_cmd = 1500.0;   // 동적 객체 멀 때 모터 명령
double g_stop_dist_dynamic = 0.7;  // 동적 객체 정지 거리 (m)
double g_slow_start_dist = 2.0;    // 이 거리부터 감속 시작 (m)

// 조향 파라미터
double g_servo_center = 0.5;       // 0~1
double g_servo_min    = 0.0;
double g_servo_max    = 1.0;
double g_servo_sign   = 1.0;       // 필요 시 방향 뒤집기
double g_right_target_px = 440.0;  // 오른쪽 차선 x 목표
double g_right_steer_gain = 0.002; // (px 오차) -> [-1,1] 조향 입력 / **이 값을 크게 해서 조향을 높여야 함.
double g_right_timeout_sec = 0.5;

// 상태
ros::Subscriber g_detect_sub;
ros::Subscriber g_centroids_sub;
ros::Subscriber g_right_centers_sub;
ros::Publisher g_motor_pub;
ros::Publisher g_servo_pub;

bool g_rotary_detected = false;
ros::Time g_last_detect_time;
double g_detect_timeout_sec = 0.5;

// 센트로이드 기반 추가 정지 조건
bool g_centroid_close = false;
ros::Time g_last_centroid_time;
double g_centroid_min_angle = 170.0;  // deg
double g_centroid_max_angle = 240.0;  // deg
double g_centroid_stop_dist = 1.0;    // meters
double g_min_dynamic_dist = std::numeric_limits<double>::infinity();
bool g_dynamic_recent = false;

// 오른쪽 차선 센터
bool g_have_right_center = false;
double g_right_center_x = 0.0;
ros::Time g_last_right_time;

inline double clamp(double x, double lo, double hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

void rotaryDetectedCB(const std_msgs::Bool::ConstPtr& msg) {
  g_rotary_detected = msg->data;
  g_last_detect_time = ros::Time::now();
}

void centroidsCB(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  g_centroid_close = false;
  g_min_dynamic_dist = std::numeric_limits<double>::infinity();
  g_dynamic_recent = false;
  const auto& data = msg->data;
  for (std::size_t i = 0; i + 1 < data.size(); i += 2) {
    const double angle = static_cast<double>(data[i]);
    const double dist = static_cast<double>(data[i + 1]);
    g_min_dynamic_dist = std::min(g_min_dynamic_dist, dist);
    if (angle >= g_centroid_min_angle && angle <= g_centroid_max_angle && dist <= g_centroid_stop_dist) {
      g_centroid_close = true;
      break;
    }
  }
  g_last_centroid_time = ros::Time::now();
  g_dynamic_recent = !data.empty();
}

void rightCentersCB(const geometry_msgs::Polygon::ConstPtr& msg) {
  g_have_right_center = false;
  if (msg->points.empty()) return;

  double sum_x = 0.0;
  for (const auto& p : msg->points) {
    sum_x += static_cast<double>(p.x);
  }
  g_right_center_x = sum_x / static_cast<double>(msg->points.size());
  g_have_right_center = true;
  g_last_right_time = ros::Time::now();
}
}  // namespace

// =====================================================
// main_node.cpp 에서 사용할 init / step 함수
// =====================================================

void mission_rotary_init(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  ROS_INFO("[mission_rotary] init");

  // 파라미터 로드
  pnh.param<std::string>("rotary_detected_topic", g_topic_rotary_detected, std::string("/rotary_detected"));
  pnh.param<std::string>("centroids_topic", g_topic_centroids, std::string("rotary/centroids"));
  pnh.param<std::string>("motor_topic", g_topic_motor, std::string("/commands/motor/speed"));
  pnh.param<std::string>("right_centers_topic", g_topic_right_centers,
                         std::string("/perception/right_window_centers_px"));
  pnh.param<std::string>("servo_topic", g_topic_servo,
                         std::string("/commands/servo/position"));

  pnh.param<double>("cruise_speed_mps", g_cruise_speed_mps, 1.0);          // 감지 없을 때 속도
  pnh.param<double>("stop_speed_mps", g_stop_speed_mps, 0.0);              // 정지 속도
  pnh.param<double>("far_motor_cmd", g_far_motor_cmd, 1500.0);             // 먼 거리 시 모터 명령
  pnh.param<double>("stop_dist_dynamic", g_stop_dist_dynamic, 0.7);        // 동적 객체 정지 거리(m)
  pnh.param<double>("slow_start_dist", g_slow_start_dist, 2.0);            // 감속 시작 거리(m)

  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);                // 모터 명령 최소
  pnh.param<double>("motor_max_cmd", g_motor_max_cmd, 2000.0);             // 모터 명령 최대
  pnh.param<double>("motor_gain", g_motor_gain, 3846.15);                  // m/s -> 명령 변환 (1000cmd=0.26m/s)
  pnh.param<double>("detect_timeout_sec", g_detect_timeout_sec, 0.5);      // 감지 타임아웃
  pnh.param<double>("centroid_min_angle_deg", g_centroid_min_angle, 170.0); // 정지 고려 각도 최소
  pnh.param<double>("centroid_max_angle_deg", g_centroid_max_angle, 240.0); // 정지 고려 각도 최대
  pnh.param<double>("centroid_stop_dist", g_centroid_stop_dist, 1.0);      // 지정 각도 내 정지 거리
  pnh.param<double>("servo_center", g_servo_center, 0.5);                  // 서보 중립
  pnh.param<double>("servo_min",    g_servo_min,    0.0);                  // 서보 최소
  pnh.param<double>("servo_max",    g_servo_max,    1.0);                  // 서보 최대
  pnh.param<double>("servo_sign",   g_servo_sign,   1.0);                  // 서보 방향 계수
  pnh.param<double>("right_target_px",  g_right_target_px, 440.0);         // 오른쪽 차선 목표 x(px)
  pnh.param<double>("right_steer_gain", g_right_steer_gain, 0.002);        // x 오차 -> 조향 게인
  pnh.param<double>("right_timeout_sec", g_right_timeout_sec, 0.5);        // 차선 정보 타임아웃

  ROS_INFO("[mission_rotary] subscribe detected='%s'",
           ros::names::resolve(g_topic_rotary_detected).c_str());
  ROS_INFO("[mission_rotary] subscribe centroids='%s' (angle %.0f~%.0f deg, dist<=%.1f m)",
           ros::names::resolve(g_topic_centroids).c_str(),
           g_centroid_min_angle, g_centroid_max_angle, g_centroid_stop_dist);
  ROS_INFO("[mission_rotary] subscribe right_centers='%s' (target_px=%.1f gain=%.4f)",
           ros::names::resolve(g_topic_right_centers).c_str(),
           g_right_target_px, g_right_steer_gain);
  ROS_INFO("[mission_rotary] publish motor='%s'",
           ros::names::resolve(g_topic_motor).c_str());
  ROS_INFO("[mission_rotary] publish servo='%s' center=%.2f",
           ros::names::resolve(g_topic_servo).c_str(), g_servo_center);

  // Pub/Sub 설정
  g_motor_pub = nh.advertise<std_msgs::Float64>(g_topic_motor, 10);
  g_servo_pub = nh.advertise<std_msgs::Float64>(g_topic_servo, 10);
  g_detect_sub = nh.subscribe(g_topic_rotary_detected, 1, rotaryDetectedCB);
  g_centroids_sub = nh.subscribe(g_topic_centroids, 1, centroidsCB);
  g_right_centers_sub = nh.subscribe(g_topic_right_centers, 1, rightCentersCB);
  g_last_detect_time = ros::Time(0);
  g_last_centroid_time = ros::Time(0);
  g_last_right_time = ros::Time(0);

  ROS_INFO("[mission_rotary] init done");
}

// main loop에서 호출
void mission_rotary_step() {
  bool detected = false;
  if (!g_last_detect_time.isZero()) {
    double dt = (ros::Time::now() - g_last_detect_time).toSec();
    detected = (dt <= g_detect_timeout_sec) && g_rotary_detected;
  }

  bool centroid_stop = false;
  bool centroids_recent = false;
  if (!g_last_centroid_time.isZero()) {
    double dt = (ros::Time::now() - g_last_centroid_time).toSec();
    centroids_recent = (dt <= g_detect_timeout_sec) && g_dynamic_recent;
    centroid_stop = centroids_recent && g_centroid_close;
  }

  // 동적 객체 거리 기반 속도: 0.7m 이내 정지, slow_start_dist부터 선형 감속
  double motor_cmd = g_far_motor_cmd;
  if (centroids_recent && std::isfinite(g_min_dynamic_dist)) {
    if (g_min_dynamic_dist <= g_stop_dist_dynamic) {
      motor_cmd = 0.0;
    } else {
      const double span = std::max(0.1, g_slow_start_dist - g_stop_dist_dynamic);
      double alpha = (g_min_dynamic_dist - g_stop_dist_dynamic) / span;
      alpha = clamp(alpha, 0.0, 1.0);
      motor_cmd = alpha * g_far_motor_cmd;
    }
  } else if (!detected) {
    // 감지 없으면 기본 크루즈 속도 사용
    motor_cmd = clamp(g_motor_gain * g_cruise_speed_mps, g_motor_min_cmd, g_motor_max_cmd);
  }
  motor_cmd = clamp(motor_cmd, g_motor_min_cmd, g_motor_max_cmd);

  // 오른쪽 차선 따라가기: 목표 x와 평균 x 차이를 조향 입력으로 사용
  double servo_hw = g_servo_center;
  bool have_right = false;
  if (!g_last_right_time.isZero()) {
    double dt = (ros::Time::now() - g_last_right_time).toSec();
    have_right = g_have_right_center && (dt <= g_right_timeout_sec);
  }

  if (have_right) {
    double err_px = g_right_target_px - g_right_center_x; // +면 오른쪽 조향
    double steer_norm = g_right_steer_gain * err_px;
    steer_norm *= g_servo_sign;
    steer_norm = clamp(steer_norm, -1.0, 1.0);

    double servo_range = std::min(g_servo_center - g_servo_min,
                                  g_servo_max - g_servo_center);
    servo_hw = g_servo_center + steer_norm * servo_range;
    servo_hw = clamp(servo_hw, g_servo_min, g_servo_max);
  }

  std_msgs::Float64 motor_msg;
  std_msgs::Float64 servo_msg;
  motor_msg.data = motor_cmd;
  servo_msg.data = servo_hw;
  g_motor_pub.publish(motor_msg);
  g_servo_pub.publish(servo_msg);

  ROS_INFO_THROTTLE(0.5,
                    "[mission_rotary] detected=%d centroids_recent=%d min_dyn_dist=%.2f motor=%.1f",
                    static_cast<int>(detected),
                    static_cast<int>(centroids_recent),
                    std::isfinite(g_min_dynamic_dist) ? g_min_dynamic_dist : -1.0,
                    motor_cmd);
}
