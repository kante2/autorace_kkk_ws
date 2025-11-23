// crosswalk_detect.cpp
// BEV binary 이미지에서 흰색 비율을 보고 횡단보도 감지를 퍼블리시한다.
// 단독 노드가 아니라 mission_crosswalk.cpp 에 포함되어 사용된다.

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <string>
#include <cstdio>

// 파라미터/상태
static std::string g_image_topic;
static std::string g_detected_topic;
static double g_white_ratio_threshold = 0.2;  // 0~1
static double g_hold_duration_sec = 9.0;      // 감지 유지 시간(메인 미션 7+2s에 맞춤)
static bool g_show_window = false;

static ros::Publisher g_detected_pub;
static ros::Subscriber g_image_sub;

static bool g_detected = false;
static ros::Time g_last_detect_time;

static double computeWhiteRatio(const cv::Mat &binary, cv::Mat &debug_img) {
  const int h = binary.rows;
  const int w = binary.cols;

  if (w <= 0 || h <= 0) {
    debug_img = cv::Mat();
    return 0.0;
  }

  const int total_pixels = w * h;
  const int total_white = cv::countNonZero(binary);
  const double white_ratio = static_cast<double>(total_white) / static_cast<double>(total_pixels);

  cv::cvtColor(binary, debug_img, cv::COLOR_GRAY2BGR);

  char text[128];
  std::snprintf(text, sizeof(text), "white_ratio=%.3f thr=%.3f", white_ratio,
                g_white_ratio_threshold);
  cv::putText(debug_img, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
              cv::Scalar(0, 255, 0), 2);

  return white_ratio;
}

static void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // 기본적으로 mono8로 기대
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  } catch (const cv_bridge::Exception &e) {
    ROS_WARN_THROTTLE(1.0, "[crosswalk_detect] cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat &binary = cv_ptr->image;
  if (binary.empty()) {
    ROS_WARN_THROTTLE(1.0, "[crosswalk_detect] empty image");
    return;
  }

  cv::Mat debug_img;
  const double white_ratio = computeWhiteRatio(binary, debug_img);

  const ros::Time now = ros::Time::now();
  const bool trigger = (white_ratio >= g_white_ratio_threshold);

  if (trigger) {
    g_detected = true;
    g_last_detect_time = now;
  } else if (g_detected) {
    const double dt = (now - g_last_detect_time).toSec();
    if (dt >= g_hold_duration_sec) {
      g_detected = false;
    }
  }

  std_msgs::Bool msg_detected;
  msg_detected.data = g_detected;
  g_detected_pub.publish(msg_detected);

  if (trigger) {
    ROS_INFO_THROTTLE(1.0, "[crosswalk_detect] TRIGGER white_ratio=%.3f thr=%.3f", white_ratio,
                      g_white_ratio_threshold);
  }
  ROS_INFO_THROTTLE(2.0, "[crosswalk_detect] detected=%d ratio=%.3f", static_cast<int>(g_detected),
                    white_ratio);

  if (g_show_window) {
    cv::Mat vis_bin, vis_dbg;
    cv::resize(binary, vis_bin, cv::Size(640, 480));
    cv::resize(debug_img, vis_dbg, cv::Size(640, 480));

    cv::imshow("crosswalk_binary", vis_bin);
    cv::imshow("crosswalk_debug", vis_dbg);
    cv::waitKey(1);
  }
}

// mission_crosswalk.cpp 에서 호출하여 서브스크라이버/퍼블리셔를 세팅한다.
void crosswalk_detect_init(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  pnh.param<std::string>("image_topic", g_image_topic, std::string("/stopline/bev_binary"));
  pnh.param<std::string>("detected_topic", g_detected_topic, std::string("/crosswalk_detected"));
  pnh.param("white_ratio_threshold", g_white_ratio_threshold, 0.2);
  pnh.param("hold_duration_sec", g_hold_duration_sec, 9.0);
  pnh.param("show_window", g_show_window, false);

  g_detected_pub = nh.advertise<std_msgs::Bool>(g_detected_topic, 1);
  g_image_sub = nh.subscribe(g_image_topic, 2, imageCallback);

  if (g_show_window) {
    cv::namedWindow("crosswalk_binary", cv::WINDOW_NORMAL);
    cv::namedWindow("crosswalk_debug", cv::WINDOW_NORMAL);
    cv::resizeWindow("crosswalk_binary", 640, 480);
    cv::resizeWindow("crosswalk_debug", 640, 480);
  }

  g_detected = false;
  g_last_detect_time = ros::Time(0);

  ROS_INFO("[crosswalk_detect] subscribing image='%s', publishing detected='%s'",
           ros::names::resolve(g_image_topic).c_str(),
           ros::names::resolve(g_detected_topic).c_str());
  ROS_INFO("[crosswalk_detect] thr=%.3f hold=%.1fs show_window=%d", g_white_ratio_threshold,
           g_hold_duration_sec, static_cast<int>(g_show_window));
}

// (optional) 현재 감지 상태를 읽을 수 있게 제공
bool crosswalk_detect_is_detected() { return g_detected; }
