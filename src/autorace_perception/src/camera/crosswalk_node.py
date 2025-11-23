// crosswalk_node.cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <vector>
#include <algorithm>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

// -------------------- 전역 상태 --------------------
ros::Publisher g_pub_is_cross_walk;  // /perception/go_straight_cmd
ros::Publisher g_pub_white_ratio;    // /perception/white_ratio

bool   g_show_window = true;
std::string g_win_bev = "crosswalk_bev";

double g_roi_top_y_ratio     = 0.60;
double g_roi_left_top_ratio  = 0.22;
double g_roi_right_top_ratio = 0.78;
double g_roi_left_bot_ratio  = -0.40;
double g_roi_right_bot_ratio = 1.40;

// HSV 범위 (노란 + 흰색 차선)
cv::Scalar g_yellow_lower(10, 80, 60);
cv::Scalar g_yellow_upper(45, 255, 255);
cv::Scalar g_white_lower(0, 0, 150);
cv::Scalar g_white_upper(179, 60, 255);

// 정지선 관련 전역 변수
double    g_white_ratio_threshold = 0.25;  // 흰색 비율 threshold
bool      g_stop_active           = false; // 현재 정지 중인지 여부
ros::Time g_stop_start_time;              // 정지 시작 시간
double    g_stop_duration         = 7.0;   // 정지 유지 시간(초)

bool      is_cross_walk           = false; // true면 "횡단보도 지나가라" 플래그

// --------------------- compute --------------------------
double computeWhiteRatio(const cv::Mat& binary)
{
  int h = binary.rows;
  int w = binary.cols;

  if (w <= 0 || h <= 0) {
    return 0.0;
  }

  const int total_pixels = w * h;
  const int total_white  = cv::countNonZero(binary);
  const double white_ratio = static_cast<double>(total_white) /
                             static_cast<double>(total_pixels);

  ROS_INFO_THROTTLE(1.0, "[crosswalk_node] white_ratio=%.3f thr=%.3f",
                    white_ratio, g_white_ratio_threshold);
  return white_ratio;
}

void compute_StopState(double white_ratio, const ros::Time& now)
{
  const bool can_start  = (!g_stop_active) &&
                          (white_ratio > g_white_ratio_threshold);
  const bool can_finish = g_stop_active &&
                          !g_stop_start_time.isZero() &&
                          (now - g_stop_start_time).toSec() >= g_stop_duration;

  if (can_start)
  {
    g_stop_active     = true;
    g_stop_start_time = now;
    ROS_INFO("[crosswalk_node] STOP TRIGGERED (white_ratio=%.3f)", white_ratio);
  }

  if (can_finish)
  {
    g_stop_active     = false;
    g_stop_start_time = ros::Time(0);
    ROS_INFO("[crosswalk_node] STOP RELEASED (duration done)");
  }
}

void pub_go_straight_cmd(const ros::Publisher& pub, bool go_straight)
{
  std_msgs::Bool msg;
  msg.data = go_straight;  // is_cross_walk 값
  pub.publish(msg);
}

void makeRoiPolygon(int h, int w, std::vector<cv::Point>& poly_out)
{
  int y_top = static_cast<int>(h * g_roi_top_y_ratio);
  int y_bot = h - 1;
  int x_lt  = static_cast<int>(w * g_roi_left_top_ratio);
  int x_rt  = static_cast<int>(w * g_roi_right_top_ratio);
  int x_lb  = static_cast<int>(w * g_roi_left_bot_ratio);
  int x_rb  = static_cast<int>(w * g_roi_right_bot_ratio);

  poly_out.clear();
  poly_out.emplace_back(x_lb, y_bot);
  poly_out.emplace_back(x_lt, y_top);
  poly_out.emplace_back(x_rt, y_top);
  poly_out.emplace_back(x_rb, y_bot);
}

cv::Mat warpToBev(const cv::Mat& bgr, const std::vector<cv::Point>& roi_poly)
{
  int h = bgr.rows;
  int w = bgr.cols;

  cv::Point2f BL = roi_poly[0];
  cv::Point2f TL = roi_poly[1];
  cv::Point2f TR = roi_poly[2];
  cv::Point2f BR = roi_poly[3];

  // y를 프레임 안으로 클리핑
  BL.y = std::max(0.f, std::min(static_cast<float>(h - 1), BL.y));
  TL.y = std::max(0.f, std::min(static_cast<float>(h - 1), TL.y));
  TR.y = std::max(0.f, std::min(static_cast<float>(h - 1), TR.y));
  BR.y = std::max(0.f, std::min(static_cast<float>(h - 1), BR.y));

  std::vector<cv::Point2f> src, dst;
  src.push_back(BL);
  src.push_back(TL);
  src.push_back(TR);
  src.push_back(BR);

  dst.push_back(cv::Point2f(0,     h - 1));
  dst.push_back(cv::Point2f(0,     0));
  dst.push_back(cv::Point2f(w - 1, 0));
  dst.push_back(cv::Point2f(w - 1, h - 1));

  cv::Mat M = cv::getPerspectiveTransform(src, dst);
  cv::Mat bev;
  cv::warpPerspective(bgr, bev, M, cv::Size(w, h),
                      cv::INTER_LINEAR,
                      cv::BORDER_CONSTANT,
                      cv::Scalar(0, 0, 0));
  return bev;
}

cv::Mat binarizeLanes(const cv::Mat& bgr)
{
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask_y, mask_w;
  cv::inRange(hsv, g_yellow_lower, g_yellow_upper, mask_y);
  cv::inRange(hsv, g_white_lower,  g_white_upper,  mask_w);

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(mask_y, mask_y, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
  cv::morphologyEx(mask_w, mask_w, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);

  cv::Mat mask;
  cv::bitwise_or(mask_y, mask_w, mask);
  return mask;
}

// 횡단보도(정지선) 인식 + 7초 정지 + 2초 플래그 로직
bool computeCrossWalk(double white_ratio, const ros::Time& now)
{
  // 1) 정지 상태 업데이트
  compute_StopState(white_ratio, now);

  // 2) 정지 상태에서 풀린 뒤 2초 동안 is_cross_walk = true
  static bool      last_stop_active      = false;
  static ros::Time go_straight_start_time;

  // "방금 정지해제 된 순간" 감지
  if (last_stop_active && !g_stop_active)
  {
    is_cross_walk          = true;
    go_straight_start_time = now;
    ROS_INFO("[crosswalk_node] crosswalk flag START (2s)");
  }

  // 플래그 유지 시간 체크 (2초 지나면 false)
  if (is_cross_walk)
  {
    double dt = (now - go_straight_start_time).toSec();
    if (dt >= 2.0)
    {
      is_cross_walk = false;
      ROS_INFO("[crosswalk_node] crosswalk flag END");
    }
  }

  // 다음 프레임을 위해 상태 저장
  last_stop_active = g_stop_active;

  // 현재 횡단보도 플래그 반환
  return is_cross_walk;
}

// -------------------- 콜백 --------------------
void imageCB(const sensor_msgs::ImageConstPtr& msg)
{
  try {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat bgr = cv_ptr->image.clone();
    if (bgr.empty()) return;

    if (bgr.channels() == 1) {
      cv::cvtColor(bgr, bgr, cv::COLOR_GRAY2BGR);
    }

    int h = bgr.rows;
    int w = bgr.cols;

    // ROI
    std::vector<cv::Point> roi_poly_pts;
    makeRoiPolygon(h, w, roi_poly_pts);

    // BEV
    cv::Mat bev_bgr = warpToBev(bgr, roi_poly_pts);

    // 이진화
    cv::Mat bev_binary = binarizeLanes(bev_bgr);

    // 흰색 비율 계산 + 퍼블리시
    double white_ratio = computeWhiteRatio(bev_binary);
    std_msgs::Float64 ratio_msg;
    ratio_msg.data = white_ratio;
    g_pub_white_ratio.publish(ratio_msg);

    // 정지/횡단보도 플래그 계산
    ros::Time now = ros::Time::now();
    bool crosswalk_flag = computeCrossWalk(white_ratio, now);

    // 현재 is_cross_walk 값을 퍼블리시
    pub_go_straight_cmd(g_pub_is_cross_walk, crosswalk_flag);

    if (g_show_window) {
      cv::imshow(g_win_bev, bev_binary);
      cv::waitKey(1);
    }

  } catch (const cv_bridge::Exception& e) {
    ROS_WARN("[crosswalk_node] cv_bridge exception: %s", e.what());
  } catch (const cv::Exception& e) {
    ROS_WARN("[crosswalk_node] OpenCV exception: %s", e.what());
  } catch (const std::exception& e) {
    ROS_WARN("[crosswalk_node] std::exception: %s", e.what());
  } catch (...) {
    ROS_WARN("[crosswalk_node] unknown exception");
  }
}

// -------------------- main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "crosswalk_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 파라미터
  pnh.param<bool>("show_window", g_show_window, true);
  pnh.param<double>("white_ratio_threshold", g_white_ratio_threshold, 0.25);
  pnh.param<double>("stop_duration",        g_stop_duration,        7.0);

  pnh.param<double>("roi_top_y_ratio",     g_roi_top_y_ratio,     0.60);
  pnh.param<double>("roi_left_top_ratio",  g_roi_left_top_ratio,  0.22);
  pnh.param<double>("roi_right_top_ratio", g_roi_right_top_ratio, 0.78);
  pnh.param<double>("roi_left_bot_ratio",  g_roi_left_bot_ratio, -0.40);
  pnh.param<double>("roi_right_bot_ratio", g_roi_right_bot_ratio, 1.40);

  ros::Subscriber img_sub =
      nh.subscribe("/usb_cam/image_rect_color", 2, imageCB);

  g_pub_is_cross_walk =
      nh.advertise<std_msgs::Bool>("/perception/go_straight_cmd", 1);
  g_pub_white_ratio =
      nh.advertise<std_msgs::Float64>("/perception/white_ratio", 1);

  if (g_show_window) {
    cv::namedWindow(g_win_bev, cv::WINDOW_NORMAL);
    cv::resizeWindow(g_win_bev, 960, 540);
  }

  ROS_INFO("crosswalk_node running...");
  ros::spin();
  return 0;
}
