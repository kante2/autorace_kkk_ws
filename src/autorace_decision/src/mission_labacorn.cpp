#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <laser_geometry/laser_geometry.h>

#include <cmath>
#include <vector>
#include <string>
#include <limits>
#include <algorithm>

// -------------------- 유틸 --------------------
inline double clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- 2D 포인트 --------------------
struct Point2D
{
  double x;
  double y;
};

struct ClusterCenters
{
  double left_x_sum  = 0.0;
  double left_y_sum  = 0.0;
  double right_x_sum = 0.0;
  double right_y_sum = 0.0;
  int    left_cnt    = 0;
  int    right_cnt   = 0;
};

// -------------------- 전역 토픽/퍼블리셔 --------------------
std::string g_topic_scan;
std::string g_topic_motor;
std::string g_topic_servo;
std::string g_topic_marker;
std::string g_topic_cloud;

ros::Publisher g_pub_motor;
ros::Publisher g_pub_servo;
ros::Publisher g_pub_marker;
ros::Publisher g_pub_cloud;

laser_geometry::LaserProjection g_projector;

// -------------------- 전역 파라미터 --------------------
// DBSCAN / ROI 파라미터
double g_eps          = 0.2;
int    g_min_samples  = 4;
double g_range_min    = 0.13;
double g_range_max    = 0.9;
double g_front_min_deg = 60.0;
double g_front_max_deg = 300.0;
double g_lane_offset_y = 0.12;   // 한쪽만 있을 때 중심 보정량

// 조향/속도 파라미터
double g_k_yaw            = 0.8;   // yaw -> 조향 gain
double g_follow_speed_mps = 1.0;   // 타겟 있을 때 속도 (m/s)
double g_min_speed_mps    = 0.7;   // (필요시 사용)

// 서보/모터 파라미터
double g_servo_center = 0.5;
double g_servo_min    = 0.0;
double g_servo_max    = 1.0;
double g_steer_sign   = -1.0; // 방향 반전

double g_motor_min_cmd = 0.0;
double g_motor_max_cmd = 2000.0;
double g_motor_gain    = 1500.0; // 1.0 m/s -> 1500 정도면 예전 1500, 1100 근사 가능

// 타임아웃
double g_scan_timeout_sec = 0.5;

// -------------------- 내부 상태 --------------------
bool      g_have_cb_time   = false;
ros::Time g_last_cb_time;

bool   g_have_target       = false;   // 현재 프레임에서 타겟이 있는지
double g_latest_steer_cmd  = 0.0;     // -1 ~ +1
double g_latest_speed_cmd  = 0.0;     // m/s

// -------------------- DBSCAN --------------------
std::vector<int> dbscan(const std::vector<Point2D>& points)
{
  const int n = static_cast<int>(points.size());
  std::vector<int> labels(n, -1);
  if (n == 0) return labels;

  int cluster_id = 0;

  auto dist = [](const Point2D& a, const Point2D& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  };

  for (int i = 0; i < n; ++i)
  {
    if (labels[i] != -1) continue;

    // i의 이웃 찾기
    std::vector<int> neighbors;
    for (int j = 0; j < n; ++j)
    {
      if (dist(points[i], points[j]) <= g_eps)
        neighbors.push_back(j);
    }

    if ((int)neighbors.size() < g_min_samples)
      continue;  // 노이즈

    ++cluster_id;
    labels[i] = cluster_id;

    std::vector<int> seed_set = neighbors;
    for (size_t k = 0; k < seed_set.size(); ++k)
    {
      int j = seed_set[k];
      if (labels[j] != -1) continue;

      labels[j] = cluster_id;

      std::vector<int> j_neighbors;
      for (int m = 0; m < n; ++m)
      {
        if (dist(points[j], points[m]) <= g_eps)
          j_neighbors.push_back(m);
      }

      if ((int)j_neighbors.size() >= g_min_samples)
      {
        seed_set.insert(seed_set.end(), j_neighbors.begin(), j_neighbors.end());
      }
    }
  }

  return labels;
}

// -------------------- 1. 스캔 포인트 필터링 --------------------
std::vector<Point2D> filterScanPoints(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  std::vector<Point2D> points;
  points.reserve(scan->ranges.size());

  double angle = scan->angle_min;

  for (auto r : scan->ranges)
  {
    // 거리 필터
    if (!std::isfinite(r) || r < g_range_min || r > g_range_max)
    {
      angle += scan->angle_increment;
      continue;
    }

    // 각도 필터 (degree 기준)
    double angle_deg = angle * 180.0 / M_PI;
    if (angle_deg < 0.0) angle_deg += 360.0;

    if (angle_deg < g_front_min_deg || angle_deg > g_front_max_deg)
    {
      angle += scan->angle_increment;
      continue;
    }

    Point2D p;
    p.x = r * std::cos(angle);
    p.y = r * std::sin(angle);
    points.push_back(p);

    angle += scan->angle_increment;
  }

  return points;
}

// -------------------- 2. 클러스터 중심 계산 + Marker 퍼블리시 --------------------
ClusterCenters computeClusterCenters(const std::vector<Point2D>& points,
                                     const std::string& frame_id,
                                     const ros::Time& stamp)
{
  ClusterCenters centers;

  if (points.empty())
    return centers;

  std::vector<int> labels = dbscan(points);
  if (labels.empty())
    return centers;

  int max_label = *std::max_element(labels.begin(), labels.end());
  if (max_label < 1)
    return centers;  // 전부 노이즈(-1)인 경우

  for (int c = 1; c <= max_label; ++c)
  {
    double cx = 0.0, cy = 0.0;
    int    count = 0;

    for (size_t i = 0; i < points.size(); ++i)
    {
      if (labels[i] == c)
      {
        cx += points[i].x;
        cy += points[i].y;
        ++count;
      }
    }
    if (count == 0) continue;

    cx /= count;
    cy /= count;

    // 좌/우 분류 (y>0: 왼쪽, y<0: 오른쪽)
    if (cy > 0.0)
    {
      centers.left_x_sum += cx;
      centers.left_y_sum += cy;
      centers.left_cnt   += 1;
    }
    else
    {
      centers.right_x_sum += cx;
      centers.right_y_sum += cy;
      centers.right_cnt   += 1;
    }

    // Marker 퍼블리시
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp    = stamp;
    marker.ns   = "cluster";
    marker.id   = c;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = cx;
    marker.pose.position.y = cy;
    marker.pose.position.z = 0.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(0.1);
    g_pub_marker.publish(marker);
  }

  return centers;
}

// -------------------- 3. 목표 좌표 계산 --------------------
bool computeTargetFromCenters(const ClusterCenters& centers,
                              double& target_x,
                              double& target_y)
{
  if (centers.left_cnt > 0 && centers.right_cnt > 0)
  {
    double left_x  = centers.left_x_sum  / centers.left_cnt;
    double left_y  = centers.left_y_sum  / centers.left_cnt;
    double right_x = centers.right_x_sum / centers.right_cnt;
    double right_y = centers.right_y_sum / centers.right_cnt;

    int total = centers.left_cnt + centers.right_cnt;

    // 한쪽에 점이 많으면 그쪽을 좀 더 반영
    target_x = (double)centers.right_cnt / total * left_x +
               (double)centers.left_cnt   / total * right_x;
    target_y = (double)centers.right_cnt / total * left_y +
               (double)centers.left_cnt   / total * right_y;

    return true;
  }
  else if (centers.left_cnt > 0)
  {
    target_x = centers.left_x_sum / centers.left_cnt;
    target_y = (centers.left_y_sum / centers.left_cnt) - g_lane_offset_y;  // 살짝 우측으로
    return true;
  }
  else if (centers.right_cnt > 0)
  {
    target_x = centers.right_x_sum / centers.right_cnt;
    target_y = (centers.right_y_sum / centers.right_cnt) + g_lane_offset_y; // 살짝 좌측으로
    return true;
  }

  // 둘 다 없음
  return false;
}

// -------------------- 4. 스캔 콜백 --------------------
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // 타임아웃용 시간 기록
  g_last_cb_time = ros::Time::now();
  g_have_cb_time = true;

  // PointCloud2 디버그용 퍼블리시 (옵션)
  if (g_pub_cloud.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 cloud;
    g_projector.projectLaser(*scan, cloud);
    cloud.header = scan->header;
    g_pub_cloud.publish(cloud);
  }

  // 1) ROI 필터링
  std::vector<Point2D> points = filterScanPoints(scan);
  if (points.empty())
  {
    g_have_target = false;
    ROS_INFO_THROTTLE(1.0, "[cluster_follower] No valid points in ROI");
    return;
  }

  // 2) 클러스터 중심 계산 + Marker
  ClusterCenters centers = computeClusterCenters(points,
                                                 scan->header.frame_id,
                                                 scan->header.stamp);

  // 3) 목표 좌표 계산
  double target_x = 0.0, target_y = 0.0;
  if (!computeTargetFromCenters(centers, target_x, target_y))
  {
    g_have_target = false;
    ROS_INFO_THROTTLE(1.0, "[cluster_follower] No clusters -> no target");
    return;
  }

  // 4) 목표 좌표 -> yaw, steer_cmd, speed_cmd
  double yaw = std::atan2(target_y, target_x);   // 앞(+x), 좌(+y) 기준

  // -1 ~ +1 범위의 조향 명령 (tanh 비선형)
  double steer_cmd = std::tanh(g_k_yaw * yaw);
  double speed_cmd = g_follow_speed_mps;         // 필요하면 yaw 크기에 따라 줄이는 로직 추가 가능

  g_latest_steer_cmd = steer_cmd;
  g_latest_speed_cmd = speed_cmd;
  g_have_target      = true;

  ROS_INFO_THROTTLE(0.5,
      "[cluster_follower][CB] target=(%.2f, %.2f), yaw=%.2f rad -> steer=%.3f, v=%.2f",
      target_x, target_y, yaw, steer_cmd, speed_cmd);
}

// -------------------- main --> mission_labacorn_step --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cluster_follower");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("cluster_follower (LaserScan -> motor/servo Float64)");

  // --- 파라미터 로드 ---

  // 토픽 이름
  pnh.param<std::string>("scan_topic",   g_topic_scan,   std::string("/scan"));
  pnh.param<std::string>("motor_topic",  g_topic_motor,  std::string("/commands/motor/speed"));
  pnh.param<std::string>("servo_topic",  g_topic_servo,  std::string("/commands/servo/position"));
  pnh.param<std::string>("marker_topic", g_topic_marker, std::string("/dbscan_lines"));
  pnh.param<std::string>("cloud_topic",  g_topic_cloud,  std::string("/scan_points"));

  ROS_INFO("[cluster_follower] subscribe scan='%s'",
           ros::names::resolve(g_topic_scan).c_str());
  ROS_INFO("[cluster_follower] publish motor='%s', servo='%s', marker='%s', cloud='%s'",
           ros::names::resolve(g_topic_motor).c_str(),
           ros::names::resolve(g_topic_servo).c_str(),
           ros::names::resolve(g_topic_marker).c_str(),
           ros::names::resolve(g_topic_cloud).c_str());

  // DBSCAN / ROI
  pnh.param<double>("eps",          g_eps,          0.2);
  pnh.param<int>   ("min_samples",  g_min_samples,  4);
  pnh.param<double>("range_min",    g_range_min,    0.13);
  pnh.param<double>("range_max",    g_range_max,    0.9);
  pnh.param<double>("front_min_deg", g_front_min_deg, 60.0);
  pnh.param<double>("front_max_deg", g_front_max_deg, 300.0);
  pnh.param<double>("lane_offset_y", g_lane_offset_y, 0.12);

  // 조향/속도
  pnh.param<double>("yaw_gain",          g_k_yaw,            0.8);
  pnh.param<double>("follow_speed_mps",  g_follow_speed_mps, 1.0);
  pnh.param<double>("min_speed_mps",     g_min_speed_mps,    0.7);

  // 서보/모터
  pnh.param<double>("servo_center", g_servo_center, 0.5);
  pnh.param<double>("servo_min",    g_servo_min,    0.0);
  pnh.param<double>("servo_max",    g_servo_max,    1.0);
  pnh.param<double>("steer_sign",   g_steer_sign,  -1.0);

  pnh.param<double>("motor_min_cmd", g_motor_min_cmd, 0.0);
  pnh.param<double>("motor_max_cmd", g_motor_max_cmd, 2000.0);
  pnh.param<double>("motor_gain",    g_motor_gain,    1500.0);

  // 타임아웃
  pnh.param<double>("scan_timeout_sec", g_scan_timeout_sec, 0.5);

  // --- Pub/Sub ---
  g_pub_motor  = nh.advertise<std_msgs::Float64>(g_topic_motor,  10);
  g_pub_servo  = nh.advertise<std_msgs::Float64>(g_topic_servo,  10);
  g_pub_marker = nh.advertise<visualization_msgs::Marker>(g_topic_marker, 10);
  g_pub_cloud  = nh.advertise<sensor_msgs::PointCloud2>(g_topic_cloud,     1);

  ros::Subscriber scan_sub = nh.subscribe(g_topic_scan, 1, scanCallback);

  // --- 메인 루프 (lane_center_controller 스타일) ---
  ros::Rate rate(15);
  while (ros::ok())
  {
    ros::spinOnce();

    ros::Time now = ros::Time::now();
    bool have_scan = false;
    if (g_have_cb_time)
    {
      double dt = (now - g_last_cb_time).toSec();
      have_scan = (dt <= g_scan_timeout_sec);
    }

    double steer_cmd = 0.0; // -1 ~ +1
    double speed_cmd = 0.0; // m/s

    if (have_scan && g_have_target)
    {
      steer_cmd = g_latest_steer_cmd;
      speed_cmd = g_latest_speed_cmd;
    }
    else
    {
      // 스캔이 없거나 타겟이 없을 때: 일단 정지
      steer_cmd = 0.0;
      speed_cmd = 0.0;

      if (!have_scan)
      {
        ROS_INFO_THROTTLE(1.0,
          "[cluster_follower] waiting LaserScan... (timeout)");
      }
      else
      {
        ROS_INFO_THROTTLE(1.0,
          "[cluster_follower] have_scan but no target -> stop");
      }
    }

    // ---- 1) 조향 변환: -1~+1 -> 서보 0~1 ----
    double steer_norm = clamp(steer_cmd, -1.0, 1.0);

    // 방향 뒤집기(필요시)
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
      "[cluster_follower][loop] have_scan=%d have_target=%d "
      "steer_cmd=%.3f servo=%.3f motor=%.1f v=%.2f",
      (int)have_scan, (int)g_have_target,
      steer_cmd, servo_hw, motor_cmd, speed_cmd);

    rate.sleep();
  }

  return 0;
}
