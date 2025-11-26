// crosswalk_node.cpp
// ----- 송도에선 yellow / 대회장은 white (파라미터로 선택) -----
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <vector>
#include <algorithm>
#include <string>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

// -------------------- 전역 상태 --------------------
ros::Publisher g_pub_crosswalk;    // /crosswalk_detected
ros::Publisher g_pub_white_ratio;  // /perception/white_ratio

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
cv::Scalar g_white_lower(0, 0, 170);
cv::Scalar g_white_upper(179, 60, 255);

// 임계값
// *********** 디버깅 *****************
double g_white_ratio_threshold = 0.30;  // 흰색 비율 threshold

// 송도/대회장 전환용 플래그
bool g_use_yellow = false;
bool g_use_white  = true;

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

  ROS_INFO_THROTTLE(1.0, "[crosswalk_node] white_ratio=%.3f thr=%.3f",white_ratio, g_white_ratio_threshold);
  return white_ratio;
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

  cv::Mat mask_y, mask_w, mask;

  // 노란선 사용
  if (g_use_yellow)
  {
    cv::inRange(hsv, g_yellow_lower, g_yellow_upper, mask_y);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask_y, mask_y, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
  }

  // 흰선 사용
  if (g_use_white)
  {
    cv::inRange(hsv, g_white_lower, g_white_upper, mask_w);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask_w, mask_w, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
  }

  // 둘 다 켰을 때
  if (!mask_y.empty() && !mask_w.empty())
  {
    cv::bitwise_or(mask_y, mask_w, mask);
  }
  else if (!mask_y.empty())
  {
    mask = mask_y;
  }
  else if (!mask_w.empty())
  {
    mask = mask_w;
  }
  else
  {
    // 둘 다 false면 그냥 0으로 (아무것도 검출 안 함)
    mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
  }

  return mask;
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

    // 비율 계산 + 퍼블리시
    double white_ratio = computeWhiteRatio(bev_binary);
    std_msgs::Float64 ratio_msg;
    ratio_msg.data = white_ratio;
    g_pub_white_ratio.publish(ratio_msg);

    // ---- simple 플래그: threshold 넘으면 true, 아니면 false ----
    bool crosswalk_detected = (white_ratio > g_white_ratio_threshold);

    std_msgs::Bool cw_msg;
    cw_msg.data = crosswalk_detected;
    g_pub_crosswalk.publish(cw_msg);

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
  pnh.param<double>("white_ratio_threshold", g_white_ratio_threshold, 0.3);

  pnh.param<double>("roi_top_y_ratio",     g_roi_top_y_ratio,     0.60);
  pnh.param<double>("roi_left_top_ratio",  g_roi_left_top_ratio,  0.22);
  pnh.param<double>("roi_right_top_ratio", g_roi_right_top_ratio, 0.78);
  pnh.param<double>("roi_left_bot_ratio",  g_roi_left_bot_ratio, -0.40);
  pnh.param<double>("roi_right_bot_ratio", g_roi_right_bot_ratio, 1.40);

  // 송도/대회장 전환용 플래그
  //  - 송도:  use_yellow_lanes=true,  use_white_lanes=false
  //  - 대회장: use_yellow_lanes=false, use_white_lanes=true
  pnh.param<bool>("use_yellow_lanes", g_use_yellow, false);
  pnh.param<bool>("use_white_lanes",  g_use_white,  true);

  std::string image_topic;
  pnh.param<std::string>("image_topic", image_topic,
                         std::string("/usb_cam/image_rect_color"));

  std::string crosswalk_topic;
  std::string white_ratio_topic;
  pnh.param<std::string>("crosswalk_topic",   crosswalk_topic,
                         std::string("/crosswalk_detected"));
  pnh.param<std::string>("white_ratio_topic", white_ratio_topic,
                         std::string("/perception/white_ratio"));

  ros::Subscriber img_sub =
      nh.subscribe(image_topic, 2, imageCB);

  g_pub_crosswalk =
      nh.advertise<std_msgs::Bool>(crosswalk_topic, 1);
  g_pub_white_ratio =
      nh.advertise<std_msgs::Float64>(white_ratio_topic, 1);

  if (g_show_window) {
    cv::namedWindow(g_win_bev, cv::WINDOW_NORMAL);
    cv::resizeWindow(g_win_bev, 960, 540);
  }

  ROS_INFO("crosswalk_node running...");
  ROS_INFO("  use_yellow_lanes = %s", g_use_yellow ? "true" : "false");
  ROS_INFO("  use_white_lanes  = %s", g_use_white  ? "true" : "false");

  ros::spin();
  return 0;
}
