// lane_color_node.cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <algorithm>

// -------------------- 전역 상태 --------------------
ros::Publisher g_pub_center_color;

bool g_show_window = true;
std::string g_win_src = "lane_color_src";
std::string g_win_bev = "lane_color_bev";

double g_roi_top_y_ratio     = 0.60;
double g_roi_left_top_ratio  = 0.22;
double g_roi_right_top_ratio = 0.78;
double g_roi_left_bot_ratio  = -0.40;
double g_roi_right_bot_ratio = 1.40;

int g_mission1_min_pixel = 500;

// 빨강(두 구간) + 파랑
cv::Scalar g_red_lower1(0,   80, 80);
cv::Scalar g_red_upper1(10, 255, 255);
cv::Scalar g_red_lower2(160, 80, 80);
cv::Scalar g_red_upper2(179, 255, 255);

cv::Scalar g_blue_lower(100, 120, 80);
cv::Scalar g_blue_upper(130, 255, 255);

// -------------------- 구조체 --------------------
struct LaneColorResult {
  int code;           // 0=none, 1=red, 2=blue
  std::string name;   // "none"/"red"/"blue"
  cv::Rect roi;
  cv::Mat mask_red;
  cv::Mat mask_blue;
};

// -------------------- 헬퍼 함수 --------------------
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

LaneColorResult mission1DetectCenterColor(const cv::Mat& bev_bgr)
{
  LaneColorResult res;
  res.code = 0;
  res.name = "none";

  int h = bev_bgr.rows;
  int w = bev_bgr.cols;

  cv::Mat hsv;
  cv::cvtColor(bev_bgr, hsv, cv::COLOR_BGR2HSV);

  // 빨강 두 구간 OR
  cv::Mat mask_red1, mask_red2, mask_red, mask_blue;
  cv::inRange(hsv, g_red_lower1, g_red_upper1, mask_red1);
  cv::inRange(hsv, g_red_lower2, g_red_upper2, mask_red2);
  cv::bitwise_or(mask_red1, mask_red2, mask_red);

  // 파랑
  cv::inRange(hsv, g_blue_lower, g_blue_upper, mask_blue);

  // 모폴로지 (노이즈 제거)
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(mask_red,  mask_red,  cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
  cv::morphologyEx(mask_blue, mask_blue, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);

  // 중앙 하단 영역
  int x1 = static_cast<int>(0.25 * w);
  int x2 = static_cast<int>(0.75 * w);
  int y1 = static_cast<int>(0.5 * h);
  int y2 = h;

  cv::Rect roi_rect(x1, y1, x2 - x1, y2 - y1);
  cv::Mat roi_red  = mask_red(roi_rect);
  cv::Mat roi_blue = mask_blue(roi_rect);

  int red_count  = cv::countNonZero(roi_red);
  int blue_count = cv::countNonZero(roi_blue);

  if (red_count > g_mission1_min_pixel || blue_count > g_mission1_min_pixel) {
    if (red_count > blue_count) {
      res.code = 1;
      res.name = "red"; // res.code = 1 -> 빨강
    } else {
      res.code = 2;
      res.name = "blue"; // res.code = 2 -> 파랑
    }
  }

  res.roi       = roi_rect;
  res.mask_red  = mask_red;
  res.mask_blue = mask_blue;
  return res;
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

    // ROI 폴리곤 & 시각화
    std::vector<cv::Point> roi_poly_pts;
    makeRoiPolygon(h, w, roi_poly_pts);

    cv::Mat src_vis = bgr.clone();
    cv::Mat overlay = bgr.clone();
    std::vector<std::vector<cv::Point>> polys;
    polys.push_back(roi_poly_pts);
    cv::fillPoly(overlay, polys, cv::Scalar(0, 255, 0));
    cv::addWeighted(overlay, 0.25, bgr, 0.75, 0.0, src_vis);
    cv::polylines(src_vis, polys, true, cv::Scalar(0, 0, 0), 2);

    // BEV
    cv::Mat bev_bgr = warpToBev(bgr, roi_poly_pts);

    // 색 판별
    LaneColorResult lc = mission1DetectCenterColor(bev_bgr);

    // 토픽 퍼블리시
    geometry_msgs::PointStamped color_msg;
    color_msg.header = msg->header;
    color_msg.header.frame_id = "lane_color";
    color_msg.point.x = static_cast<double>(lc.code); // 0,1,2
    color_msg.point.y = 0.0;
    color_msg.point.z = 0.0;
    g_pub_center_color.publish(color_msg);

    // 디버그 이미지
    cv::Mat dbg = bev_bgr.clone();
    cv::Scalar color_box(0, 255, 255);
    if (lc.code == 1)      color_box = cv::Scalar(0, 0, 255);   // red
    else if (lc.code == 2) color_box = cv::Scalar(255, 0, 0);   // blue
    cv::rectangle(dbg, lc.roi, color_box, 2);

    auto put = [&](const std::string& txt, int y)
    {
      cv::putText(dbg, txt, cv::Point(10, y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6,
                  cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    };
    put("Mission1 CenterColor: " + lc.name +
          " (code=" + std::to_string(lc.code) + ")", 24);

    if (g_show_window) {
      cv::imshow(g_win_src, src_vis);
      cv::imshow(g_win_bev, dbg);
      cv::waitKey(1);
    }

  } catch (const cv_bridge::Exception& e) {
    ROS_WARN("[lane_color_node] cv_bridge exception: %s", e.what());
  } catch (const cv::Exception& e) {
    ROS_WARN("[lane_color_node] OpenCV exception: %s", e.what());
  } catch (const std::exception& e) {
    ROS_WARN("[lane_color_node] std::exception: %s", e.what());
  } catch (...) {
    ROS_WARN("[lane_color_node] unknown exception");
  }
}

// -------------------- main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_color_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<bool>("show_window", g_show_window, true);
  pnh.param<int>("mission1_min_pixel", g_mission1_min_pixel, 500);

  pnh.param<double>("roi_top_y_ratio",     g_roi_top_y_ratio,     0.60);
  pnh.param<double>("roi_left_top_ratio",  g_roi_left_top_ratio,  0.22);
  pnh.param<double>("roi_right_top_ratio", g_roi_right_top_ratio, 0.78);
  pnh.param<double>("roi_left_bot_ratio",  g_roi_left_bot_ratio, -0.40);
  pnh.param<double>("roi_right_bot_ratio", g_roi_right_bot_ratio, 1.40);

  ros::Subscriber img_sub = nh.subscribe("/usb_cam/image_rect_color", 2, imageCB);

  g_pub_center_color =
      nh.advertise<geometry_msgs::PointStamped>("/perception/center_color_px", 1);

  if (g_show_window) {
    cv::namedWindow(g_win_src, cv::WINDOW_NORMAL);
    cv::resizeWindow(g_win_src, 960, 540);
    cv::namedWindow(g_win_bev, cv::WINDOW_NORMAL);
    cv::resizeWindow(g_win_bev, 960, 540);
  }

  ROS_INFO("lane_color_node running...");
  ros::spin();
  return 0;
}
