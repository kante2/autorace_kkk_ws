#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "autorace_control/controller_lane_logic.hpp"
#include "autorace_control/controller_obstacle_dbscan_logic.hpp"

// -------------------- lane 전역 실제 정의 --------------------
// (이미 네가 쓰는 그대로)

// ... (g_topic_center_point, g_motor_topic, g_pub_motor, g_pub_servo 등등)

// -------------------- obstacle 전역 실제 정의 --------------------
std::string g_obs_scan_topic;
std::string g_obs_marker_topic;
std::string g_obs_cloud_topic;
double      g_obs_eps;
int         g_obs_min_samples;
double      g_obs_range_min;
double      g_obs_range_max;
double      g_obs_front_min_deg;
double      g_obs_front_max_deg;
double      g_obs_lane_offset_y;
double      g_obs_yaw_gain;
double      g_obs_follow_speed_mps;
double      g_obs_min_speed_mps;
double      g_obs_scan_timeout_sec;

bool        g_obs_have_scan_time     = false;
ros::Time   g_obs_last_scan_time;
bool        g_obs_have_target        = false;
double      g_obs_latest_steer_cmd   = 0.0;
double      g_obs_latest_speed_cmd   = 0.0;

ros::Publisher g_pub_obs_marker;
ros::Publisher g_pub_obs_cloud;

// -------------------- main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_labacorn");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // lane 파라미터
  loadParams(pnh);

  // obstacle 파라미터
  loadParams_obstacle_dbscan(pnh);

  // obstacle용 publisher advertise
  g_pub_obs_marker = nh.advertise<visualization_msgs::Marker>(g_obs_marker_topic, 10);
  g_pub_obs_cloud  = nh.advertise<sensor_msgs::PointCloud2>(g_obs_cloud_topic, 1);

  // scan 구독
  ros::Subscriber scan_sub =
      nh.subscribe(g_obs_scan_topic, 1, CB_obstacle_scan);

  // lane 구독/퍼블리셔 (네가 쓰던 그대로)
  // ...

  ros::Rate rate(15);
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Time now = ros::Time::now();

    // 여기서 lane vs obstacle 선택, 또는 obstacle 모드일 때:
    //   g_obs_have_scan_time, g_obs_last_scan_time, g_obs_have_target, g_obs_latest_steer_cmd/speed_cmd
    //   를 이용해서 최종 steer_cmd/speed_cmd 결정
    //
    // 그리고 나서 publish_motor_commands(steer_cmd, speed_cmd) 호출하는 구조로 가면 돼

    rate.sleep();
  }
}
