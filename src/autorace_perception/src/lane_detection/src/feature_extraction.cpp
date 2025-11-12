#include "lane_detection/feature_extraction.hpp"

#include <opencv2/imgproc.hpp>
namespace lane_detection {

namespace {
constexpr int kLaneThreshold = 9000;
}  // namespace

LaneFeatureExtractor::LaneFeatureExtractor() {
    initCurrentLane();
}

void LaneFeatureExtractor::initCurrentLane() {
    current_lane_ = "right";
}

std::vector<int> LaneFeatureExtractor::estimateStopLine(cv::Mat& binary,
                                                        int img_y,
                                                        int threshold) const {
    (void)img_y;  // 현재 로직에서는 사용하지 않지만 시그니처 유지

    std::vector<int> stop_line;
    if (binary.empty()) {
        return stop_line;
    }

    std::vector<int> indices;
    indices.reserve(binary.rows);

    for (int y = 0; y < binary.rows; ++y) {
        cv::Mat row = binary.row(y);
        int count = cv::countNonZero(row);
        if (count > threshold) {
            indices.push_back(y);
        }
    }

    if (!indices.empty()) {
        int min_y = indices.front();
        int max_y = indices.back();
        cv::Rect roi(0, min_y, binary.cols, std::max(1, max_y - min_y + 1));
        binary(roi).setTo(0);
        stop_line = {min_y, max_y};
    }

    return stop_line;
}

std::string LaneFeatureExtractor::estimateLaneMode(const cv::Mat& bev_bgr) {
    if (bev_bgr.empty()) {
        return current_lane_;
    }

    cv::Mat hsv;
    cv::cvtColor(bev_bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Scalar yellow_lower(18, 140, 120);
    cv::Scalar yellow_upper(34, 255, 255);
    cv::Mat yellow_mask;
    cv::inRange(hsv, yellow_lower, yellow_upper, yellow_mask);

    int width = yellow_mask.cols;
    int height = yellow_mask.rows;
    if (width == 0 || height == 0) {
        return current_lane_;
    }

    cv::Rect left_rect(0, 0, width / 2, height);
    cv::Rect right_rect(width / 2, 0, width - width / 2, height);

    int left_yellow_count = cv::countNonZero(yellow_mask(left_rect));
    int right_yellow_count = cv::countNonZero(yellow_mask(right_rect));

    if (left_yellow_count > kLaneThreshold) {
        current_lane_ = "left";
    } else if (right_yellow_count > kLaneThreshold) {
        current_lane_ = "right";
    }

    return current_lane_;
}

bool LaneFeatureExtractor::detectOutRotary(const cv::Mat& yellow_binary) const {
    if (yellow_binary.empty()) {
        return false;
    }

    int height = yellow_binary.rows;
    int width = yellow_binary.cols;

    int x1 = static_cast<int>(0.68 * width);
    int x2 = width;
    int y1 = 0;
    int y2 = static_cast<int>(0.42 * height);
    if (x1 >= x2 || y1 >= y2) {
        return false;
    }

    cv::Rect roi(x1, y1, x2 - x1, y2 - y1);
    cv::Mat region = yellow_binary(roi);
    int yellow_count = cv::countNonZero(region);
    return yellow_count > 0;
}

}  // namespace lane_detection