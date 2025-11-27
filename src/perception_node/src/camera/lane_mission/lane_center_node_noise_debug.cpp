// lane_center_node_noise_debugging.cpp

// lane_center_node.cpp 와 같은 로직이면서, 색상 튜닝 값만 변경된 파일이다.
// 여기서  ROSBAG + lane_center_node_noise_debugging.cpp 을 실행하여, 노란색만 잘 잡는지 체크하고
// 값이 완료되면 팀원과 값을 공유하고 lane_center_node.cpp에 최종 튜닝값을 주입한다.

// 실제 주행의 차선 perception부는 lane_center_node.cpp 이다.
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include <std_msgs/Float64.h>

// -------------------- 전역 상태 --------------------
ros::Publisher g_pub_center_point;
ros::Publisher g_pub_curvature;   // 곡률 퍼블리셔

bool g_show_window = true;
std::string g_win_src = "lane_src_debug";
std::string g_win_bev = "lane_bev_binary";

// 파라미터
double g_lane_width_px = 340.0;

int    g_num_windows       = 12;
int    g_window_margin     = 80;
int    g_minpix_recenter   = 50;
int    g_min_lane_sep      = 60;
double g_center_ema_alpha  = 0.8;

double g_roi_top_y_ratio     = 0.60;
double g_roi_left_top_ratio  = 0.22;
double g_roi_right_top_ratio = 0.78;
double g_roi_left_bot_ratio  = -0.40;
double g_roi_right_bot_ratio = 1.40;

// 외부 조명 있는 환경에서 "노란 차선만" 강건하게 잡도록 약간 타이트하게 설정
// 필요하면 H/S/V 값은 직접 다시 튜닝해서 쓰면 됨

// **************** tuning ************************
// Hue; 색상, Saturation;채도, Value;명도
// 현재값: (15, 100, 80), (40, 255, 255) -> 10 근처: 삘강 주황, 32~45 초록으로 넘어감

// 디버깅 했던거 정리 최적 범위 찾기(lower 높이고, upper 줄이기)
// H: 10~20 주황,  20~30 노랑,  30~40 노랑+연두
// S: 0~30: 무채색, 30~80 누런 색, 80~150: 진한색, 150~255: 쨍한색
// V: 0~50: 매우 어두움;그림자//50~100: 어두운 색;실내 어두운 곳//100~180:보통밝기//180~255: 아주 밝음;흰색, 강한 조명

// 1. (18, 80, 70), (32, 230, 230)
// 2. (15, 80, 70), (30, 255, 230)

cv::Scalar g_yellow_lower(15, 100, 80);   // H, S, V (lower)
cv::Scalar g_yellow_upper(40, 255, 255);  // H, S, V (upper)

//cv::Scalar g_yellow_lower(18, 80, 70);   // H, S, V (lower)
//cv::Scalar g_yellow_upper(32, 230, 230);  // H, S, V (upper)

//cv::Scalar g_yellow_lower(15, 80, 70);   // H, S, V (lower)
//cv::Scalar g_yellow_upper(30, 255, 235);  // H, S, V (upper)



// -------------------- 헬퍼 함수들 --------------------
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

// -------------------- 노란 차선만 이진화 --------------------
cv::Mat binarizeLanes(const cv::Mat& bgr)
{
  // 조명 노이즈 조금 줄이기 위해 약한 블러
  cv::Mat bgr_blur;
  cv::GaussianBlur(bgr, bgr_blur, cv::Size(5, 5), 0);

  cv::Mat hsv;
  cv::cvtColor(bgr_blur, hsv, cv::COLOR_BGR2HSV);

  // 노란색 차선만 필터링 ** <- 이 관련 값이 튜닝 대상,, 
  cv::Mat mask_y;
  cv::inRange(hsv, g_yellow_lower, g_yellow_upper, mask_y);

  // 노이즈 제거 (opening 2번 정도)
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(mask_y, mask_y, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 2);

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

// 간단한 곡률 계산 (폴리라인의 대표 3점으로 근사)
double computeCurvatureFromCenters(const std::vector<cv::Point2f>& centers)
{
  if (centers.size() < 3) {
    return 0.0;
  }

  cv::Point2f p1 = centers.front();
  cv::Point2f p2 = centers[centers.size() / 2];
  cv::Point2f p3 = centers.back();

  double x1 = p1.x, y1 = p1.y;
  double x2 = p2.x, y2 = p2.y;
  double x3 = p3.x, y3 = p3.y;

  double a = std::hypot(x2 - x1, y2 - y1);
  double b = std::hypot(x3 - x2, y3 - y2);
  double c = std::hypot(x3 - x1, y3 - y1);

  double area2 = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1); // 2*area with sign
  double A = std::fabs(area2) * 0.5;

  if (A < 1e-6 || a < 1e-6 || b < 1e-6 || c < 1e-6) {
    return 0.0;
  }

  double R = (a * b * c) / (4.0 * A);
  double kappa = 1.0 / R;

  // 방향에 따라 부호 (좌회전/우회전)
  double sign = (area2 > 0.0) ? 1.0 : -1.0;
  return sign * kappa;
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

    // 카메라 프레임 중심 (디버그)
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

    // 3) 이진화 (노란 차선만)
    cv::Mat bev_binary = binarizeLanes(bev_bgr);

    // 4) 슬라이딩 윈도우
    cv::Mat debug_img;
    std::vector<cv::Point2f> left_centers, right_centers;
    runSlidingWindowCollectCenters(bev_binary, debug_img,
                                   left_centers, right_centers);

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

    // 6) 곡률 계산 (좌/우 중 포인트 많은 쪽 기준)
    std::vector<cv::Point2f> lane_pts;
    if (left_centers.size() >= right_centers.size())
      lane_pts = left_centers;
    else
      lane_pts = right_centers;

    double curvature = computeCurvatureFromCenters(lane_pts);
    std_msgs::Float64 curv_msg;
    curv_msg.data = curvature;
    g_pub_curvature.publish(curv_msg);

    // 디버그 텍스트
    auto put = [&](const std::string& txt, int y)
    {
      cv::putText(debug_img, txt, cv::Point(10, y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6,
                  cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    };
    put("Left centers:  " + std::to_string(left_centers.size()), 24);
    put("Right centers: " + std::to_string(right_centers.size()), 48);
    put("Curvature (px^-1): " + std::to_string(curvature), 72);

    // 7) 화면 출력
    if (g_show_window) {
      cv::Mat canvas;
      cv::Mat src_resized, dbg_resized;
      cv::resize(src_vis, src_resized, cv::Size(w, h));
      cv::resize(debug_img, dbg_resized, cv::Size(w, h));
      cv::hconcat(src_resized, dbg_resized, canvas);

      // cv::imshow(g_win_src, canvas);
      // cv::imshow(g_win_bev, bev_binary);
      cv::waitKey(1);
    }

  } catch (const cv_bridge::Exception& e) {
    ROS_WARN("[lane_center_node] cv_bridge exception: %s", e.what());
  } catch (const cv::Exception& e) {
    ROS_WARN("[lane_center_node] OpenCV exception: %s", e.what());
  } catch (const std::exception& e) {
    ROS_WARN("[lane_center_nodNANce] std::exception: %s", e.what());
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
  pnh.param<double>("lane_width_px", g_lane_width_px, 340.0);

  pnh.param<int>("num_windows",      g_num_windows,      12);
  pnh.param<int>("window_margin",    g_window_margin,    80);
  pnh.param<int>("minpix_recenter",  g_minpix_recenter,  50);
  pnh.param<int>("min_lane_sep",     g_min_lane_sep,     60);
  pnh.param<double>("center_ema_alpha", g_center_ema_alpha, 0.8);

  pnh.param<double>("roi_top_y_ratio",     g_roi_top_y_ratio,     0.60);
  pnh.param<double>("roi_left_top_ratio",  g_roi_left_top_ratio,  0.22);
  pnh.param<double>("roi_right_top_ratio", g_roi_right_top_ratio, 0.78);
  pnh.param<double>("roi_left_bot_ratio",  g_roi_left_bot_ratio, -0.40);
  pnh.param<double>("roi_right_bot_ratio", g_roi_right_bot_ratio, 1.40);

  // Pub/Sub
  ros::Subscriber img_sub = nh.subscribe("/usb_cam/image_rect_color", 2, imageCB);

  g_pub_center_point = nh.advertise<geometry_msgs::PointStamped>("/perception/center_point_px", 1);
  g_pub_curvature    = nh.advertise<std_msgs::Float64>("/perception/curvature_center", 1);

  if (g_show_window) {
    cv::namedWindow(g_win_src, cv::WINDOW_NORMAL);
    cv::resizeWindow(g_win_src, 960, 540);
    cv::namedWindow(g_win_bev, cv::WINDOW_NORMAL);
    cv::resizeWindow(g_win_bev, 960, 540);
  }

  ROS_INFO("lane_center_node running (yellow-lane only)...");
  ros::spin();
  return 0;
}
