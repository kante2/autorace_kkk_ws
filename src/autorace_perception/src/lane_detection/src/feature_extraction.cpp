#include "lane_detection/feature_extraction.hpp"

#include <opencv2/imgproc.hpp>
namespace lane_detection {

namespace {

constexpr int kLaneThreshold = 9000;  // 좌/우 영역별 최소 픽셀 수 기준

}  // namespace

LaneFeatureExtractor::LaneFeatureExtractor() {
    initCurrentLane();
}

void LaneFeatureExtractor::initCurrentLane() {
    current_lane_ = "right";  // 초기 모드는 우측 차선 주행으로 고정
}

std::vector<int> LaneFeatureExtractor::estimateStopLine(cv::Mat& binary,
                                                        int img_y,
                                                        int threshold) const {
    (void)img_y;  // 현재 로직에서는 사용하지 않지만 시그니처 유지

    std::vector<int> stop_line;
    if (binary.empty()) {
        return stop_line;
    }

    // 노란 정지선 픽셀 수가 임계값을 넘는 행 인덱스를 모두 수집
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
        cv::Rect roi(0, min_y, binary.cols, std::max(1, max_y - min_y + 1));  // 정지선 영역
        binary(roi).setTo(0);  // 정지선 영역을 0으로 비워 이후 차선 검출에 영향이 없도록 함
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

    cv::Scalar white_lower(0, 0, 192);
    cv::Scalar white_upper(179, 64, 255);
    cv::Mat white_mask;
    cv::inRange(hsv, white_lower, white_upper, white_mask);  // 좌·우 차선이 모두 흰색이므로 흰 마스크 사용

    int width = white_mask.cols;
    int height = white_mask.rows;
    if (width == 0 || height == 0) {
        return current_lane_;
    }

    cv::Rect left_rect(0, 0, width / 2, height);
    cv::Rect right_rect(width / 2, 0, width - width / 2, height);

    // 좌우 흰 차선 픽셀 수를 비교해 현재 차선 모드를 결정한다.
    int left_white_count = cv::countNonZero(white_mask(left_rect));
    int right_white_count = cv::countNonZero(white_mask(right_rect));

    if (left_white_count > kLaneThreshold) {  // 좌측 영역에 흰 픽셀이 몰리면 좌측 차선 주행으로 판단
        current_lane_ = "left";
    } else if (right_white_count > kLaneThreshold) {  // 우측 영역 픽셀이 많으면 우측 차선 모드
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
