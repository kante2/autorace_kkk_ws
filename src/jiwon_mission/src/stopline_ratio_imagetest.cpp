#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>

// ================== 전역 변수 ==================

// 흰(또는 관심) 픽셀 비율 threshold (정지선/횡단보도 판정)
double g_white_ratio_threshold = 0.3;

// 정지 상태 관리
bool      g_stop_active     = false;
ros::Time g_stop_start_time;
double    g_stop_duration   = 7.0;   // [sec]

// HSV에서 노란색 범위 (이미지에서 타겟 영역 추출용 – 그대로 사용)
cv::Scalar g_yellow_lower(15, 80, 80); 
cv::Scalar g_yellow_upper(40, 255, 255);

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
    ROS_INFO("[stopline_node] STOP TRIGGERED (white_ratio=%.3f, thr=%.3f)",
             white_ratio, g_white_ratio_threshold);
  }

  if (can_finish)
  {
    g_stop_active     = false;
    g_stop_start_time = ros::Time(0);
    ROS_INFO("[stopline_node] STOP RELEASED (duration done)");
  }

  ROS_INFO("[stopline_node] g_stop_active = %s",
           g_stop_active ? "TRUE" : "FALSE");
}

// ================== main: 이미지 1장으로 테스트 ==================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "stopline_yellowratio_imagetest");
  ros::NodeHandle nh("~");  // private node handle

  // 1) rosparam에서 image_path 및 파라미터 읽기
  std::string image_path;
  nh.param<std::string>("image_path", image_path, std::string(""));

  nh.param("white_ratio_threshold", g_white_ratio_threshold, 0.3);
  nh.param("stop_duration",         g_stop_duration,         7.0);

  if (image_path.empty())
  {
    ROS_ERROR("[test] image_path param is empty!");
    return 1;
  }

  ROS_INFO("[test] image_path = %s", image_path.c_str());
  ROS_INFO("[test] white_ratio_thr=%.3f, stop_duration=%.1f",
           g_white_ratio_threshold, g_stop_duration);

  // 2) 이미지 읽기
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  if (img.empty())
  {
    ROS_ERROR("Failed to read image: %s", image_path.c_str());
    return 1;
  }

  int h = img.rows;
  int w = img.cols;
  ROS_INFO("[test] Loaded image: (w=%d, h=%d)", w, h);

  // 3) BGR -> HSV -> 노란색 inRange -> binary
  cv::Mat hsv;
  cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

  cv::Mat yellow_binary;
  cv::inRange(hsv, g_yellow_lower, g_yellow_upper, yellow_binary);

  // 노이즈 줄이기 (선택)
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(yellow_binary, yellow_binary,
                   cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
  cv::morphologyEx(yellow_binary, yellow_binary,
                   cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 1);

  // 4) white ratio 계산
  double white_ratio = computeWhiteRatio(yellow_binary);
  ROS_INFO("[test] final white_ratio = %.3f", white_ratio);

  // 디버그용 ratio 텍스트가 올라간 이미지 (원하면 끄면 됨)
  cv::Mat debug_ratio;
  cv::cvtColor(yellow_binary, debug_ratio, cv::COLOR_GRAY2BGR);
  char text[256];
  std::snprintf(text, sizeof(text),
                "white_ratio=%.3f thr=%.3f",
                white_ratio, g_white_ratio_threshold);
  cv::putText(debug_ratio, text,
              cv::Point(10, 30),
              cv::FONT_HERSHEY_SIMPLEX, 0.7,
              cv::Scalar(0, 255, 0), 2);

  // 5) STOP / 시간 경과 테스트
  ros::Time t0 = ros::Time::now();
  ROS_INFO("[test] == First call (t0) ==");
  compute_StopState(white_ratio, t0);

  // 3초 후 (아직 7초 안 지났으니까 stop 유지)
  ros::Duration(3.0).sleep();
  ros::Time t1 = ros::Time::now();
  ROS_INFO("[test] == Second call (~t0+3sec) ==");
  compute_StopState(white_ratio, t1);

  // 추가로 5초 더 (총 8초 후, stop 해제)
  ros::Duration(5.0).sleep();
  ros::Time t2 = ros::Time::now();
  ROS_INFO("[test] == Third call (~t0+8sec) ==");
  compute_StopState(white_ratio, t2);

  ROS_INFO("[test] Final g_stop_active = %s",
           g_stop_active ? "TRUE" : "FALSE");

  // 6) 시각화
  cv::imshow("img", img);
  cv::imshow("yellow_binary", yellow_binary);
  cv::imshow("debug_ratio", debug_ratio);
  cv::waitKey(0);

  return 0;
}
