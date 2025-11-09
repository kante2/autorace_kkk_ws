#include "lane_detection/sliding_window.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace lane_detection {

SlidingWindow::SlidingWindow()
    : image_height_(0),
      window_height_(0),
      margin_(80),
      minpix_(100),
      alpha_(0.8),
      min_sep_(40),
      left_blocked_flag_(0),
      right_blocked_flag_(0),
      left_lane_start_(160),
      right_lane_start_(480) {}

void SlidingWindow::init() {
    left_blocked_flag_ = 0;
    right_blocked_flag_ = 0;
    left_lane_points_.clear();
    right_lane_points_.clear();
    nonzero_points_.clear();
}

void SlidingWindow::setImageHeight(int img_y) {
    image_height_ = img_y;
}

SlidingWindowResult SlidingWindow::slidingWindowAdaptive(const cv::Mat& binary_img,
                                                         int nwindows,
                                                         int margin,
                                                         int minpix) {
    SlidingWindowResult result;
    if (binary_img.empty()) {
        return result;
    }

    cv::Mat binary_color;
    cv::cvtColor(binary_img, binary_color, cv::COLOR_GRAY2BGR);

    init();

    margin_ = margin;
    minpix_ = minpix;

    int height = binary_img.rows;
    int width = binary_img.cols;
    int safe_width = std::max(1, width);
    int half_height = height / 2;
    int default_left = std::clamp(safe_width / 4, 0, safe_width - 1);
    int default_right = std::clamp(3 * safe_width / 4, 0, safe_width - 1);

    // 하단 절반 영역 히스토그램을 사용해 좌/우 차선 시작점을 자동 추정
    cv::Range bottom_range(half_height, height);
    cv::Mat bottom_half = binary_img(bottom_range, cv::Range::all());
    cv::Mat histogram;

    if (!bottom_half.empty()) {
        cv::reduce(bottom_half, histogram, 0, cv::REDUCE_SUM, CV_32S);
    }

    if (histogram.empty() || histogram.cols == 0) {
        left_lane_start_ = default_left;
        right_lane_start_ = default_right;
    } else {
        int midpoint = histogram.cols / 2;
        if (midpoint == 0) {
            left_lane_start_ = default_left;
            right_lane_start_ = default_right;
        } else {
            cv::Mat left_hist = histogram.colRange(0, midpoint);
            cv::Mat right_hist = histogram.colRange(midpoint, histogram.cols);

            double left_max_val = 0.0;
            double right_max_val = 0.0;
            int left_max_idx = 0;
            int right_max_idx = 0;

            cv::minMaxIdx(left_hist, nullptr, &left_max_val, nullptr, &left_max_idx);
            cv::minMaxIdx(right_hist, nullptr, &right_max_val, nullptr, &right_max_idx);

            left_lane_start_ = std::clamp(left_max_idx, 0, safe_width - 1);
            right_lane_start_ = std::clamp(right_max_idx + midpoint, 0, safe_width - 1);

            if (left_max_val <= 0.0) {
                left_lane_start_ = default_left;
            }
            if (right_max_val <= 0.0) {
                right_lane_start_ = default_right;
            }
        }
    }

    window_height_ = height / std::max(1, nwindows);

    cv::findNonZero(binary_img, nonzero_points_);

    int prev_left_margin = margin_;
    int prev_right_margin = margin_;

    int leftx_current = left_lane_start_;
    int leftx_prev = left_lane_start_;
    int lefty_current = height - window_height_ / 2;

    int rightx_current = right_lane_start_;
    int rightx_prev = right_lane_start_;
    int righty_current = height - window_height_ / 2;

    for (int i = 0; i < nwindows; ++i) {
        if (left_blocked_flag_ < 7) {
            slidingWindowLaneCalculation(leftx_current,
                                         lefty_current,
                                         left_blocked_flag_,
                                         prev_left_margin,
                                         leftx_prev,
                                         binary_color,
                                         true,
                                         rightx_current,
                                         right_blocked_flag_ < 7,
                                         min_sep_,
                                         cv::Scalar(0, 255, 0));
        }

        if (right_blocked_flag_ < 7) {
            slidingWindowLaneCalculation(rightx_current,
                                         righty_current,
                                         right_blocked_flag_,
                                         prev_right_margin,
                                         rightx_prev,
                                         binary_color,
                                         false,
                                         leftx_current,
                                         left_blocked_flag_ < 7,
                                         min_sep_,
                                         cv::Scalar(255, 0, 0));
        }
    }

    result.visualized = binary_color;
    result.left_lane = left_lane_points_;
    result.right_lane = right_lane_points_;

    return result;
}

void SlidingWindow::slidingWindowLaneCalculation(int& x_current,
                                                 int& y_current,
                                                 int& flag,
                                                 int& prev_margin,
                                                 int& x_prev,
                                                 cv::Mat& visualized,
                                                 bool is_left_lane,
                                                 int other_x,
                                                 bool other_valid,
                                                 int min_sep,
                                                 const cv::Scalar& color) {
    int half_window = window_height_ / 2;
    int win_y_low = std::max(0, y_current - half_window);
    int win_y_high = std::min(visualized.rows - 1, y_current + half_window);
    int win_x_low = std::max(0, x_current - margin_ - prev_margin);
    int win_x_high = std::min(visualized.cols - 1, x_current + margin_ + prev_margin);

    cv::rectangle(visualized,
                  cv::Point(win_x_low, win_y_low),
                  cv::Point(win_x_high, win_y_high),
                  color,
                  2);

    std::vector<int> good_indices;
    good_indices.reserve(nonzero_points_.size());
    for (int idx = 0; idx < static_cast<int>(nonzero_points_.size()); ++idx) {
        const auto& pt = nonzero_points_[idx];
        if (pt.y >= win_y_low && pt.y < win_y_high && pt.x >= win_x_low && pt.x < win_x_high) {
            good_indices.push_back(idx);
        }
    }

    if (static_cast<int>(good_indices.size()) > minpix_) {
        int min_x = visualized.cols - 1;
        int max_x = 0;
        double sum_x = 0.0;
        for (int id : good_indices) {
            int x = nonzero_points_[id].x;
            min_x = std::min(min_x, x);
            max_x = std::max(max_x, x);
            sum_x += static_cast<double>(x);
        }

        if ((max_x - min_x) > 100) {
            y_current = std::max(0, y_current - window_height_);
            flag += 1;
            return;
        }

        double mean_x = sum_x / static_cast<double>(good_indices.size());
        double delta = mean_x + (mean_x - static_cast<double>(x_prev));
        x_current = static_cast<int>(alpha_ * mean_x + (1.0 - alpha_) * delta);
        y_current = std::max(0, y_current - window_height_);
        prev_margin = std::min((max_x - min_x) / 4, 20);
        x_prev = x_current;

        if (other_valid && std::abs(x_current - other_x) < min_sep) {
            flag += 1;
            return;
        }

        cv::circle(visualized, cv::Point(x_current, y_current), 5, cv::Scalar(0, 0, 255), cv::FILLED);
        flag = 0;

        if (is_left_lane) {
            left_lane_points_.emplace_back(x_current, y_current);
        } else {
            right_lane_points_.emplace_back(x_current, y_current);
        }
    } else {
        y_current = std::max(0, y_current - window_height_);
        flag += 1;
    }
}

}  // namespace lane_detection