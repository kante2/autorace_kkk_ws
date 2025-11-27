// gate_camera_node.cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <algorithm>

// -------------------- 전역 상태 --------------------

// 디버그 창
bool g_show_window = true;
std::string g_win_name = "gate_camera_debug";

// ROI 비율 (파이썬 코드 기준)
// 가로: 20% ~ 80% (중앙 60%), 세로: 위쪽 50% (0 ~ 0.5)
double g_roi_x0_ratio = 0.20;
double g_roi_x1_ratio = 0.80;
double g_roi_y1_ratio = 0.50;   // y0 = 0, y1 = 0.5*h

// 임계값 (퍼센트)
double g_yellow_thresh_percent = 2.5;

// 퍼블리셔 (gate 노란색 검출 여부만)
ros::Publisher g_pub_gate_flag;

// HSV 범위 (노란색 두 구간)
cv::Scalar g_yellow_lower_1(15, 100, 80);
cv::Scalar g_yellow_upper_1(40, 255, 255);

cv::Scalar g_yellow_lower_2(18, 100, 110);
cv::Scalar g_yellow_upper_2(38, 255, 230);

// -------------------- 헬퍼 함수 --------------------
cv::Mat binarizeLanes(const cv::Mat &bgr,
                      const cv::Scalar &lower,
                      const cv::Scalar &upper)
{
  cv::Mat bgr_blur, hsv, mask;
  cv::GaussianBlur(bgr, bgr_blur, cv::Size(5, 5), 0);
  cv::cvtColor(bgr_blur, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, lower, upper, mask);

  // 작은 노이즈 제거용 morphology
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 2);

  return mask;
}

cv::Mat putLabel(const cv::Mat &img, const std::string &text)
{
  cv::Mat out = img.clone();
  cv::putText(out, text, cv::Point(20, 60),
              cv::FONT_HERSHEY_SIMPLEX,
              3.0,
              cv::Scalar(255, 255, 255),
              5,
              cv::LINE_AA);
  return out;
}

// -------------------- 콜백 --------------------
void imageCB(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    cv_bridge::CvImageConstPtr cv_ptr =
        cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat bgr = cv_ptr->image;
    if (bgr.empty())
      return;

    int h = bgr.rows;
    int w = bgr.cols;

    // === ROI 설정 ===
    int x0 = static_cast<int>(g_roi_x0_ratio * w);
    int x1 = static_cast<int>(g_roi_x1_ratio * w);
    int y0 = 0;
    int y1 = static_cast<int>(g_roi_y1_ratio * h);

    x0 = std::max(0, std::min(x0, w - 1));
    x1 = std::max(0, std::min(x1, w));
    y0 = std::max(0, std::min(y0, h - 1));
    y1 = std::max(0, std::min(y1, h));

    if (x1 <= x0 || y1 <= y0)
    {
      ROS_WARN_THROTTLE(1.0, "[gate_camera_node] invalid ROI, check ratios");
      return;
    }

    int roi_w = x1 - x0;
    int roi_h = y1 - y0;
    double total_pixels = static_cast<double>(roi_w * roi_h);

    // --- 각 HSV 범위별로 이진화 ---
    struct RangeInfo
    {
      std::string label;
      cv::Scalar lower;
      cv::Scalar upper;
    };

    std::vector<RangeInfo> ranges = {
        {"#1 ", g_yellow_lower_1, g_yellow_upper_1},
        {"#2 ", g_yellow_lower_2, g_yellow_upper_2},
    };

    cv::Mat combined_mask_full = cv::Mat::zeros(h, w, CV_8UC1);
    std::vector<cv::Mat> mask_imgs;
    mask_imgs.reserve(ranges.size());

    for (const auto &r : ranges)
    {
      cv::Mat mask_full = binarizeLanes(bgr, r.lower, r.upper);

      cv::Rect roi_rect(x0, y0, roi_w, roi_h);
      cv::Mat roi_part = mask_full(roi_rect);

      int nonzero = cv::countNonZero(roi_part);
      double ratio = (nonzero / total_pixels) * 100.0;

      // ROI 바깥은 0, ROI 안만 살린 마스크
      cv::Mat mask_roi_full = cv::Mat::zeros(mask_full.size(), mask_full.type());
      roi_part.copyTo(mask_roi_full(roi_rect));

      // 전체 OR 마스크 업데이트
      cv::bitwise_or(combined_mask_full, mask_roi_full, combined_mask_full);

      // 디버그용 라벨
      cv::Mat mask_bgr;
      cv::cvtColor(mask_roi_full, mask_bgr, cv::COLOR_GRAY2BGR);
      cv::Mat labeled = putLabel(mask_bgr,
                                 r.label + cv::format("%.2f%%", ratio));
      mask_imgs.push_back(labeled);
    }

    // COMBINED 비율 계산
    cv::Rect roi_rect(x0, y0, roi_w, roi_h);
    cv::Mat combined_roi = combined_mask_full(roi_rect);
    int combined_nonzero = cv::countNonZero(combined_roi);
    double combined_ratio = (combined_nonzero / total_pixels) * 100.0;

    // ----------------- 토픽 퍼블리시 -----------------
    bool detected = (combined_ratio >= g_yellow_thresh_percent);
    std_msgs::Bool flag_msg;
    flag_msg.data = detected;
    g_pub_gate_flag.publish(flag_msg);

    ROS_INFO_THROTTLE(1.0,
                      "[gate_camera_node] yellow ratio = %.2f%% (threshold=%.2f%%, detected=%d)",
                      combined_ratio, g_yellow_thresh_percent, detected ? 1 : 0);

    // ----------------- 시각화 -----------------
    if (g_show_window)
    {
      // 위: 원본 + ROI 박스 + 전체 퍼센트
      cv::Mat bgr_vis = bgr.clone();
      cv::rectangle(bgr_vis,
                    cv::Point(x0, y0),
                    cv::Point(x1 - 1, y1 - 1),
                    cv::Scalar(0, 0, 255), 3);
      cv::Mat top_row = putLabel(
          bgr_vis,
          cv::format("Original BGR (ROI yellow: %.2f%%)", combined_ratio));

      // 아래: 각 마스크
      std::vector<cv::Mat> mask_resized;
      for (auto &m : mask_imgs)
      {
        cv::Mat resized;
        cv::resize(m, resized, cv::Size(w, h));
        mask_resized.push_back(resized);
      }
      cv::Mat bottom_row;
      if (!mask_resized.empty())
      {
        cv::hconcat(mask_resized, bottom_row);
      }
      else
      {
        bottom_row = cv::Mat::zeros(top_row.size(), top_row.type());
      }

      // 가로 길이 맞추기
      int h_top = top_row.rows;
      int w_top = top_row.cols;
      int h_bot = bottom_row.rows;
      int w_bot = bottom_row.cols;

      if (w_bot < w_top)
      {
        cv::Mat pad = cv::Mat::zeros(h_bot, w_top - w_bot, bottom_row.type());
        cv::hconcat(bottom_row, pad, bottom_row);
      }
      else if (w_bot > w_top)
      {
        cv::Mat pad = cv::Mat::zeros(h_top, w_bot - w_top, top_row.type());
        cv::hconcat(top_row, pad, top_row);
      }

      cv::Mat canvas_all;
      cv::vconcat(top_row, bottom_row, canvas_all);

      cv::imshow(g_win_name, canvas_all);
      cv::waitKey(1);
    }
  }
  catch (const cv_bridge::Exception &e)
  {
    ROS_WARN("[gate_camera_node] cv_bridge exception: %s", e.what());
  }
  catch (const cv::Exception &e)
  {
    ROS_WARN("[gate_camera_node] OpenCV exception: %s", e.what());
  }
  catch (const std::exception &e)
  {
    ROS_WARN("[gate_camera_node] std::exception: %s", e.what());
  }
  catch (...)
  {
    ROS_WARN("[gate_camera_node] unknown exception");
  }
}

// -------------------- main --------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "gate_camera_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 파라미터
  pnh.param<bool>("show_window", g_show_window, true);
  pnh.param<double>("yellow_ratio_threshold", g_yellow_thresh_percent, 2.5);

  pnh.param<double>("roi_x0_ratio", g_roi_x0_ratio, 0.20);
  pnh.param<double>("roi_x1_ratio", g_roi_x1_ratio, 0.80);
  pnh.param<double>("roi_y1_ratio", g_roi_y1_ratio, 0.50);

  std::string image_topic;
  pnh.param<std::string>("image_topic",
                         image_topic,
                         std::string("/usb_cam/image_rect_color"));

  // 토픽 이름은 고정: /perception/gate_yellow_detected
  g_pub_gate_flag =
      nh.advertise<std_msgs::Bool>("/perception/gate_yellow_detected", 1);

  ros::Subscriber img_sub =
      nh.subscribe(image_topic, 1, imageCB);

  if (g_show_window)
  {
    cv::namedWindow(g_win_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(g_win_name, 1280, 720);
  }

  ROS_INFO("gate_camera_node running...");
  ros::spin();

  return 0;
}
