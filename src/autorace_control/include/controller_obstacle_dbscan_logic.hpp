#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

// -------------------- extern 전역 (mission_labacorn에서 실제 정의) --------------------

// 토픽 이름들
extern std::string g_obs_scan_topic;
extern std::string g_obs_marker_topic;
extern std::string g_obs_cloud_topic;

// DBSCAN / ROI 파라미터
extern double g_obs_eps;
extern int    g_obs_min_samples;
extern double g_obs_range_min;
extern double g_obs_range_max;
extern double g_obs_front_min_deg;
extern double g_obs_front_max_deg;
extern double g_obs_lane_offset_y;

// Yaw -> 조향/속도 파라미터
extern double g_obs_yaw_gain;
extern double g_obs_follow_speed_mps;
extern double g_obs_min_speed_mps;

// Timeout
extern double    g_obs_scan_timeout_sec;

// 상태 변수 (scan / target)
extern bool      g_obs_have_scan_time;
extern ros::Time g_obs_last_scan_time;
extern bool      g_obs_have_target;
extern double    g_obs_latest_steer_cmd;   // -1 ~ +1
extern double    g_obs_latest_speed_cmd;   // m/s

// 디버깅/시각화 퍼블리셔 (mission_labacorn에서 advertise)
extern ros::Publisher g_pub_obs_marker;
extern ros::Publisher g_pub_obs_cloud;


// -------------------- 함수 프로토타입 --------------------

// YAML + 파라미터 서버에서 obstacle DBSCAN 관련 파라미터 로드
//  - pnh는 일반적으로 ros::NodeHandle("~")
void loadParams_obstacle_dbscan(ros::NodeHandle& pnh);

// LaserScan 콜백
//  - /scan (또는 YAML에서 지정한 토픽) 구독용
//  - 내부에서 g_obs_latest_steer_cmd / g_obs_latest_speed_cmd 업데이트
void CB_obstacle_scan(const sensor_msgs::LaserScan::ConstPtr& msg);
