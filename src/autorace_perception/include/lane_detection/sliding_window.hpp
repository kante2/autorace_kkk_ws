#pragma once

#include <opencv2/core.hpp>

#include <vector>

namespace lane_detection {

struct SlidingWindowResult {
    cv::Mat visualized;
    std::vector<cv::Point> left_lane;
    std::vector<cv::Point> right_lane;
};

/**
 * @brief 단일 색상(노란 차선) 이진 영상에서 좌/우 차선을 동시에 추적한다.
 *
 * Python `sliding_window.py`의 `SlidingWindow` 클래스를 포팅하였다.
 */
class SlidingWindow {
public:
    SlidingWindow();

    void setImageHeight(int img_y);

    SlidingWindowResult slidingWindowAdaptive(const cv::Mat& binary_img,
                                              int nwindows = 12,
                                              int margin = 80,
                                              int minpix = 100);

private:
    void init();

    void slidingWindowLaneCalculation(int& x_current,
                                      int& y_current,
                                      int& flag,
                                      int& prev_margin,
                                      int& x_prev,
                                      cv::Mat& visualized,
                                      bool is_left_lane,
                                      int other_x,
                                      bool other_valid,
                                      int min_sep,
                                      const cv::Scalar& color);

    int image_height_;
    int window_height_;
    int margin_;
    int minpix_;
    double alpha_;
    int min_sep_;

    int left_blocked_flag_;
    int right_blocked_flag_;

    int left_lane_start_;
    int right_lane_start_;

    std::vector<cv::Point> nonzero_points_;
    std::vector<cv::Point> left_lane_points_;
    std::vector<cv::Point> right_lane_points_;
};

}  // namespace lane_detection