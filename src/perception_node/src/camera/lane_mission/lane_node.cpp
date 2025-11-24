#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>  

// -------------------- 전역 상태 --------------------
ros::Publisher g_pub_center_point;
ros::Publisher g_pub_center_color;
ros::Publisher g_pub_is_cross_walk;  // 횡단보도 플래그
ros::Publisher g_pub_white_ratio;    // 흰색 비율 디버그용
ros::Publisher g_pub_left_centers;   // 슬라이딩 윈도우 좌측 포인트 목록
ros::Publisher g_pub_right_centers;  // 슬라이딩 윈도우 우측 포인트 목록

bool g_show_window = true;
std::string g_win_src = "src_with_roi";
std::string g_win_bev = "bev_binary_and_windows";
bool g_swap_lane_stop_colors = false;  // false: lane=white, stop=yellow (기본 테스트 환경)

// init ----------------------------------------------
bool is_cross_walk = false;          // true면 "횡단보도 지나가라" 플래그

// 파라미터
double g_lane_width_px = 340.0;

int    g_num_windows       = 12;
int    g_window_margin     = 80;
int    g_minpix_recenter   = 50;
int    g_min_lane_sep      = 60;
double g_center_ema_alpha  = 0.8;

double g_roi_top_y_ratio     = 0.60;
double g_roi_left_top_ratio  = 0.10;
double g_roi_right_top_ratio = 0.90;
double g_roi_left_bot_ratio  = -0.40;
double g_roi_right_bot_ratio = 1.40;

int g_mission1_min_pixel = 500;

// HSV 범위
cv::Scalar g_yellow_lower(10, 80, 60);
cv::Scalar g_yellow_upper(45, 255, 255);
cv::Scalar g_white_lower(0, 0, 175);
cv::Scalar g_white_upper(179, 40, 255);

// 빨강(두 구간) + 파랑
cv::Scalar g_red_lower1(0,   80, 80);
cv::Scalar g_red_upper1(10, 255, 255);
cv::Scalar g_red_lower2(160, 80, 80);
cv::Scalar g_red_upper2(179, 255, 255);

cv::Scalar g_blue_lower(100, 120, 80);
cv::Scalar g_blue_upper(130, 255, 255);

// 정지선 관련 전역 변수
double    g_white_ratio_threshold = 0.25;  // 흰색 비율 threshold
bool      g_stop_active           = false; // 현재 정지 중인지 여부
ros::Time g_stop_start_time;              // 정지 시작 시간
double    g_stop_duration         = 7.0;   // 정지 유지 시간(초)

// -------------------- 구조체 정의 --------------------
struct LaneColorResult {
  int code;           // 0=none, 1=red, 2=blue
  std::string name;   // "none"/"red"/"blue"
  cv::Rect roi;
  cv::Mat mask_red;
  cv::Mat mask_blue;
};

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

  ROS_INFO_THROTTLE(1.0, "[stopline_node] white_ratio=%.3f thr=%.3f",
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
    ROS_INFO("[stopline_node] STOP TRIGGERED (white_ratio=%.3f)", white_ratio);
  }

  if (can_finish)
  {
    g_stop_active     = false;
    g_stop_start_time = ros::Time(0);
    ROS_INFO("[stopline_node] STOP RELEASED (duration done)");
  }
}

// -------------------- 헬퍼 함수들 --------------------
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

// 흰색 차선만 이진화
cv::Mat binarizeWhiteLanes(const cv::Mat& bgr)
{
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask_w;
  cv::inRange(hsv, g_white_lower, g_white_upper, mask_w);

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(mask_w, mask_w, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
  return mask_w;
}

// 노란 정지선/횡단보도만 이진화
cv::Mat binarizeYellowStopline(const cv::Mat& bgr)
{
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask_y;
  cv::inRange(hsv, g_yellow_lower, g_yellow_upper, mask_y);

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(mask_y, mask_y, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
  return mask_y;
}

void runSlidingWindowCollectCenters(const cv::Mat& binary_mask,
                                    cv::Mat& debug_img,
                                    std::vector<cv::Point2f>& left_window_centers,
                                    std::vector<cv::Point2f>& right_window_centers)
{
  int h = binary_mask.rows;
  int w = binary_mask.cols;

  cv::cvtColor(binary_mask, debug_img, cv::COLOR_GRAY2BGR);

  // 하단 절반 히스토그램
  cv::Mat lower = binary_mask.rowRange(h / 2, h);
  cv::Mat hist;
  cv::reduce(lower, hist, 0, cv::REDUCE_SUM, CV_32S);  // 1 x w

  int midpoint   = w / 2;
  int left_base  = -1;
  int right_base = -1;

  // left_base
  {
    int max_val = 0;
    for (int x = 0; x < midpoint; ++x) {
      int v = hist.at<int>(0, x);
      if (v > max_val) {
        max_val = v;
        left_base = x;
      }
    }
  }
  // right_base
  {
    int max_val = 0;
    for (int x = midpoint; x < w; ++x) {
      int v = hist.at<int>(0, x);
      if (v > max_val) {
        max_val = v;
        right_base = x;
      }
    }
  }

  int window_height = h / g_num_windows;
  int left_current  = left_base;
  int right_current = right_base;

  std::vector<cv::Point> nz_pts;
  cv::findNonZero(binary_mask, nz_pts);  // (x,y)

  left_window_centers.clear();
  right_window_centers.clear();

  std::vector<int> left_indices;
  std::vector<int> right_indices;

  for (int win = 0; win < g_num_windows; ++win) {
    int y_low  = h - (win + 1) * window_height;
    int y_high = h - win * window_height;

    // 창 그리기
    if (left_current >= 0) {
      cv::rectangle(debug_img,
                    cv::Point(left_current - g_window_margin, y_low),
                    cv::Point(left_current + g_window_margin, y_high),
                    cv::Scalar(255, 0, 0), 2);
    }
    if (right_current >= 0) {
      cv::rectangle(debug_img,
                    cv::Point(right_current - g_window_margin, y_low),
                    cv::Point(right_current + g_window_margin, y_high),
                    cv::Scalar(255, 0, 0), 2);
    }

    std::vector<int> good_left;
    std::vector<int> good_right;

    for (size_t i = 0; i < nz_pts.size(); ++i) {
      int px = nz_pts[i].x;
      int py = nz_pts[i].y;

      if (py >= y_low && py < y_high) {
        if (left_current >= 0 &&
            px >= left_current - g_window_margin &&
            px <  left_current + g_window_margin) {
          good_left.push_back(static_cast<int>(i));
        }
        if (right_current >= 0 &&
            px >= right_current - g_window_margin &&
            px <  right_current + g_window_margin) {
          good_right.push_back(static_cast<int>(i));
        }
      }
    }

    // 좌/우 너무 붙으면 한쪽 억제
    if (left_current >= 0 && right_current >= 0) {
      if (std::abs(left_current - right_current) < g_min_lane_sep) {
        if (good_left.size() < good_right.size()) {
          good_left.clear();
        } else {
          good_right.clear();
        }
      }
    }

    left_indices.insert(left_indices.end(),  good_left.begin(),  good_left.end());
    right_indices.insert(right_indices.end(), good_right.begin(), good_right.end());

    int y_center = (y_low + y_high) / 2;

    // left
    if (!good_left.empty()) {
      double sum_x = 0.0;
      for (int idx : good_left) sum_x += nz_pts[idx].x;
      double x_mean_left = sum_x / good_left.size();
      left_window_centers.emplace_back(static_cast<float>(x_mean_left),
                                       static_cast<float>(y_center));
      cv::circle(debug_img,
                 cv::Point(static_cast<int>(x_mean_left), y_center),
                 4, cv::Scalar(0, 0, 255), -1);
    }
    // right
    if (!good_right.empty()) {
      double sum_x = 0.0;
      for (int idx : good_right) sum_x += nz_pts[idx].x;
      double x_mean_right = sum_x / good_right.size();
      right_window_centers.emplace_back(static_cast<float>(x_mean_right),
                                        static_cast<float>(y_center));
      cv::circle(debug_img,
                 cv::Point(static_cast<int>(x_mean_right), y_center),
                 4, cv::Scalar(0, 255, 255), -1);
    }

    // EMA 업데이트
    if (!good_left.empty() && left_current >= 0) {
      double sum_x = 0.0;
      for (int idx : good_left) sum_x += nz_pts[idx].x;
      double mean_x = sum_x / good_left.size();
      left_current = static_cast<int>(g_center_ema_alpha * left_current +
                                      (1.0 - g_center_ema_alpha) * mean_x);
    }
    if (!good_right.empty() && right_current >= 0) {
      double sum_x = 0.0;
      for (int idx : good_right) sum_x += nz_pts[idx].x;
      double mean_x = sum_x / good_right.size();
      right_current = static_cast<int>(g_center_ema_alpha * right_current +
                                       (1.0 - g_center_ema_alpha) * mean_x);
    }
  }

  // 색칠 (디버그)
  for (int idx : left_indices) {
    int px = std::max(0, std::min(w - 1, nz_pts[idx].x));
    int py = std::max(0, std::min(h - 1, nz_pts[idx].y));
    debug_img.at<cv::Vec3b>(py, px) = cv::Vec3b(0, 0, 255);
  }
  for (int idx : right_indices) {
    int px = std::max(0, std::min(w - 1, nz_pts[idx].x));
    int py = std::max(0, std::min(h - 1, nz_pts[idx].y));
    debug_img.at<cv::Vec3b>(py, px) = cv::Vec3b(0, 255, 0);
  }
}

bool computeCenterPoint(const std::vector<cv::Point2f>& left_window_centers,
                        const std::vector<cv::Point2f>& right_window_centers,
                        cv::Point2f& center_out)
{
  auto side_mean = [](const std::vector<cv::Point2f>& centers,
                      double& y_mean, double& x_mean) -> bool
  {
    if (centers.empty()) return false;
    double sy = 0.0, sx = 0.0;
    for (const auto& p : centers) {
      sx += p.x;
      sy += p.y;
    }
    y_mean = sy / centers.size();
    x_mean = sx / centers.size();
    return true;
  };

  double left_y = 0.0, left_x = 0.0;
  double right_y = 0.0, right_x = 0.0;

  bool has_left  = side_mean(left_window_centers,  left_y,  left_x);
  bool has_right = side_mean(right_window_centers, right_y, right_x);

  double half_w = 0.5 * g_lane_width_px;

  if (has_left && has_right) {
    double cy = 0.5 * (left_y + right_y);
    double cx = 0.5 * (left_x + right_x);
    center_out = cv::Point2f(static_cast<float>(cx),
                             static_cast<float>(cy));
    return true;
  }

  if (has_left) {
    double cy = left_y;
    double cx = left_x + half_w;
    center_out = cv::Point2f(static_cast<float>(cx),
                             static_cast<float>(cy));
    return true;
  }

  if (has_right) {
    double cy = right_y;
    double cx = right_x - half_w;
    center_out = cv::Point2f(static_cast<float>(cx),
                             static_cast<float>(cy));
    return true;
  }

  return false;
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

    // 카메라 프레임 중심
    int cx_cam = w / 2;
    int cy_cam = h / 2;

    // 1) ROI 폴리곤 & 시각화
    std::vector<cv::Point> roi_poly_pts;
    makeRoiPolygon(h, w, roi_poly_pts);

    cv::Mat src_vis = bgr.clone();
    cv::Mat overlay = bgr.clone();
    std::vector<std::vector<cv::Point>> polys;
    polys.push_back(roi_poly_pts);
    cv::fillPoly(overlay, polys, cv::Scalar(0, 255, 0));
    cv::addWeighted(overlay, 0.25, bgr, 0.75, 0.0, src_vis);
    cv::polylines(src_vis, polys, true, cv::Scalar(0, 0, 0), 2);

    // 카메라 프레임 중심 표시 (보라색 점)
    cv::circle(src_vis, cv::Point(cx_cam, cy_cam), 6, cv::Scalar(255, 0, 255), -1);

    // 2) BEV
    cv::Mat bev_bgr = warpToBev(bgr, roi_poly_pts);

    // 3) 이진화: 차선과 정지선/횡단보도 분리
    cv::Mat lane_mask;
    cv::Mat stop_mask;
    if (g_swap_lane_stop_colors) {
      // 대회 환경: 차선=노랑, 정지선=흰색
      lane_mask = binarizeYellowStopline(bev_bgr);
      stop_mask = binarizeWhiteLanes(bev_bgr);
    } else {
      // 기본 테스트 환경: 차선=흰색, 정지선=노랑
      lane_mask = binarizeWhiteLanes(bev_bgr);
      stop_mask = binarizeYellowStopline(bev_bgr);
    }

    // 4) 슬라이딩 윈도우
    cv::Mat debug_img;
    std::vector<cv::Point2f> left_centers, right_centers;
    runSlidingWindowCollectCenters(lane_mask, debug_img,
                                   left_centers, right_centers);

    // 4-1) 좌/우 윈도우 중심 좌표 배열 퍼블리시 (픽셀 단위)
    auto publish_centers = [](const std::vector<cv::Point2f>& centers,
                              ros::Publisher& pub,
                              const std_msgs::Header& header)
    {
      (void)header; // Polygon에는 헤더가 없어 직접 넣지 않음을 명시
      geometry_msgs::Polygon poly;
      poly.points.reserve(centers.size());
      for (const auto& p : centers) {
        geometry_msgs::Point32 pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = 0.0f;
        poly.points.push_back(pt);
      }
      // Polygon에는 헤더가 없어 std_msgs::Header를 직접 넣을 수 없으므로
      // frame 정보가 필요하다면 PointStamped 배열 등 다른 메시지로 변경 고려.
      pub.publish(poly);
    };
    publish_centers(left_centers,  g_pub_left_centers,  msg->header);
    publish_centers(right_centers, g_pub_right_centers, msg->header);

    // 5) 차선 중심 퍼블리시
    cv::Point2f center_pt;
    if (computeCenterPoint(left_centers, right_centers, center_pt)) {
      geometry_msgs::PointStamped pt_msg;
      pt_msg.header = msg->header;
      pt_msg.header.frame_id = "bev";
      pt_msg.point.x = center_pt.x;
      pt_msg.point.y = center_pt.y;
      pt_msg.point.z = 0.0;
      g_pub_center_point.publish(pt_msg);

      cv::circle(debug_img,
                 cv::Point(static_cast<int>(center_pt.x),
                           static_cast<int>(center_pt.y)),
                 6, cv::Scalar(255, 0, 255), -1);
    }

    // 6) Mission_1: 차로 색 판별 + 퍼블리시
    LaneColorResult lc = mission1DetectCenterColor(bev_bgr);

    geometry_msgs::PointStamped color_msg;
    color_msg.header = msg->header;
    color_msg.header.frame_id = "lane_color";
    color_msg.point.x = static_cast<double>(lc.code); // 0,1,2
    color_msg.point.y = 0.0;
    color_msg.point.z = 0.0;
    g_pub_center_color.publish(color_msg);

    // 디버그 ROI 박스
    cv::Scalar color_box(0, 255, 255);
    if (lc.code == 1)      color_box = cv::Scalar(0, 0, 255); // red
    else if (lc.code == 2) color_box = cv::Scalar(255, 0, 0); // blue
    cv::rectangle(debug_img, lc.roi, color_box, 2);

    // 텍스트
    auto put = [&](const std::string& txt, int y)
    {
      cv::putText(debug_img, txt, cv::Point(10, y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6,
                  cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    };
    put("Left centers:  " + std::to_string(left_centers.size()), 24);
    put("Right centers: " + std::to_string(right_centers.size()), 48);
    put("Mission1 CenterColor: " + lc.name +
          " (code=" + std::to_string(lc.code) + ")", 72);

    // 7) 화면 출력
    if (g_show_window) {
      cv::Mat canvas;
      cv::Mat src_resized, dbg_resized;
      cv::resize(src_vis, src_resized, cv::Size(w, h));
      cv::resize(debug_img, dbg_resized, cv::Size(w, h));
      cv::hconcat(src_resized, dbg_resized, canvas);

      cv::imshow(g_win_src, canvas);
      cv::imshow(g_win_bev, lane_mask);
      cv::waitKey(1);
    }

  } catch (const cv_bridge::Exception& e) {
    ROS_WARN("[lane_center_node] cv_bridge exception: %s", e.what());
  } catch (const cv::Exception& e) {
    ROS_WARN("[lane_center_node] OpenCV exception: %s", e.what());
  } catch (const std::exception& e) {
    ROS_WARN("[lane_center_node] std::exception: %s", e.what());
  } catch (...) {
    ROS_WARN("[lane_center_node] unknown exception");
  }
}

// -------------------- main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_center_node");
  ros::NodeHandle nh;      // global
  ros::NodeHandle pnh("~");// private

  // 파라미터 로드
  pnh.param<bool>("show_window", g_show_window, true);
  pnh.param<bool>("swap_lane_stop_colors", g_swap_lane_stop_colors, false);
  pnh.param<double>("lane_width_px", g_lane_width_px, 340.0);

  pnh.param<int>("num_windows",      g_num_windows,      12);
  pnh.param<int>("window_margin",    g_window_margin,    80);
  pnh.param<int>("minpix_recenter",  g_minpix_recenter,  50);
  pnh.param<int>("min_lane_sep",     g_min_lane_sep,     80); // 60 -> 80
  pnh.param<double>("center_ema_alpha", g_center_ema_alpha, 0.8);

  pnh.param<double>("roi_top_y_ratio",     g_roi_top_y_ratio,     0.60);
  pnh.param<double>("roi_left_top_ratio",  g_roi_left_top_ratio,  0.10);
  pnh.param<double>("roi_right_top_ratio", g_roi_right_top_ratio, 0.85);
  pnh.param<double>("roi_left_bot_ratio",  g_roi_left_bot_ratio, -0.45);
  pnh.param<double>("roi_right_bot_ratio", g_roi_right_bot_ratio, 1.40);

  pnh.param<int>("mission1_min_pixel", g_mission1_min_pixel, 500);

  pnh.param<double>("white_ratio_threshold", g_white_ratio_threshold, 0.25);
  pnh.param<double>("stop_duration",        g_stop_duration,        7.0);

  // Pub/Sub
  ros::Subscriber img_sub = nh.subscribe("/usb_cam/image_rect_color", 1, imageCB);

  g_pub_center_point = nh.advertise<geometry_msgs::PointStamped>("/perception/center_point_px", 1);
  g_pub_center_color = nh.advertise<geometry_msgs::PointStamped>("/perception/center_color_px", 1); // 0=none,1=red,2=blue
  g_pub_is_cross_walk  = nh.advertise<std_msgs::Bool>("/perception/go_straight_cmd", 1);
  g_pub_white_ratio    = nh.advertise<std_msgs::Float64>("/perception/white_ratio", 1);
  g_pub_left_centers   = nh.advertise<geometry_msgs::Polygon>("/perception/left_window_centers_px", 1);
  g_pub_right_centers  = nh.advertise<geometry_msgs::Polygon>("/perception/right_window_centers_px", 1);

  if (g_show_window) {
    cv::namedWindow(g_win_src, cv::WINDOW_NORMAL);
    cv::resizeWindow(g_win_src, 960, 540);
    cv::namedWindow(g_win_bev, cv::WINDOW_NORMAL);
    cv::resizeWindow(g_win_bev, 960, 540);
  }

  ROS_INFO("lane_center_node running...");
  ros::spin();
  return 0;
}
