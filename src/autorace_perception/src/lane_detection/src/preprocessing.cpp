#include "lane_detection/preprocessing.hpp"

#include <opencv2/imgproc.hpp>

#include <array>

namespace lane_detection {

namespace {

constexpr float kWarpTargetHeight = 480.0f;

cv::Mat warpWithPoints(const cv::Mat& img,
                       const std::array<cv::Point2f, 4>& src_points,
                       const std::array<cv::Point2f, 4>& dst_points,
                       int img_x,
                       int img_y) {
    // 시야 영역 사다리꼴을 직사각형으로 펴서 도로를 수직 투영(BEV)하는 공통 도우미
    if (img.empty()) {
        return cv::Mat();
    }

    cv::Mat transform = cv::getPerspectiveTransform(src_points.data(), dst_points.data());
    cv::Mat warped;
    cv::warpPerspective(img, warped, transform, cv::Size(img_x, img_y));
    return warped;
}

}  // namespace

CameraPreprocessor::CameraPreprocessor() {
    initColor();
}

void CameraPreprocessor::initColor() {
    // Python 버전에서 사용하던 HSV 범위를 그대로 재현
    yellow_lower_ = cv::Scalar(15, 128, 0);
    yellow_upper_ = cv::Scalar(40, 255, 255);
    white_lower_ = cv::Scalar(0, 0, 192);
    white_upper_ = cv::Scalar(179, 64, 255);
}

cv::Mat CameraPreprocessor::bevWarp(const cv::Mat& img, int img_y, int img_x) const {
    constexpr float delta = 25.0f;

    // 일반 주행 환경에서 차선을 감시하는 사다리꼴 ROI 정의
    std::array<cv::Point2f, 4> src_points{
        cv::Point2f(-delta, 400.0f),
        cv::Point2f(180.0f, 300.0f),
        cv::Point2f(static_cast<float>(img_x - 180), 300.0f),
        cv::Point2f(static_cast<float>(img_x) + delta, 400.0f)};

    int x_div_8 = img_x / 8;
    std::array<cv::Point2f, 4> dst_points{
        cv::Point2f(static_cast<float>(x_div_8), kWarpTargetHeight),
        cv::Point2f(static_cast<float>(x_div_8), 0.0f),
        cv::Point2f(static_cast<float>(x_div_8 * 7), 0.0f),
        cv::Point2f(static_cast<float>(x_div_8 * 7), kWarpTargetHeight)};

    return warpWithPoints(img, src_points, dst_points, img_x, img_y);
}

cv::Mat CameraPreprocessor::bevWarpRotary(const cv::Mat& img, int img_y, int img_x) const {
    constexpr float delta = 25.0f;

    // 로터리 진입 시 더 넓은 영역을 잡도록 원본 사다리꼴을 조정
    std::array<cv::Point2f, 4> src_points{
        cv::Point2f(-delta, kWarpTargetHeight),
        cv::Point2f(291.0f, 260.0f),
        cv::Point2f(static_cast<float>(img_x - 291), 260.0f),
        cv::Point2f(static_cast<float>(img_x) + delta, kWarpTargetHeight)};

    int x_div_8 = img_x / 8;
    std::array<cv::Point2f, 4> dst_points{
        cv::Point2f(static_cast<float>(x_div_8), kWarpTargetHeight),
        cv::Point2f(static_cast<float>(x_div_8), 0.0f),
        cv::Point2f(static_cast<float>(x_div_8 * 7), 0.0f),
        cv::Point2f(static_cast<float>(x_div_8 * 7), kWarpTargetHeight)};

    return warpWithPoints(img, src_points, dst_points, img_x, img_y);
}

void CameraPreprocessor::detectColorYellowAndWhite(const cv::Mat& img_bgr,
                                                   const cv::Mat& img_hsv,
                                                   cv::Mat& yellow_filtered,
                                                   cv::Mat& white_filtered) const {
    if (img_bgr.empty() || img_hsv.empty()) {
        yellow_filtered.release();
        white_filtered.release();
        return;
    }

    cv::Mat yellow_mask;
    cv::inRange(img_hsv, yellow_lower_, yellow_upper_, yellow_mask);
    cv::bitwise_and(img_bgr, img_bgr, yellow_filtered, yellow_mask);

    // 정지선·차선 모두 동일 프레임에서 처리할 수 있도록 흰색도 동시에 필터링
    cv::Mat white_mask;
    cv::inRange(img_hsv, white_lower_, white_upper_, white_mask);
    cv::bitwise_and(img_bgr, img_bgr, white_filtered, white_mask);
}

void CameraPreprocessor::binaryImagesYellowAndWhite(const cv::Mat& yellow_filtered,
                                                    const cv::Mat& white_filtered,
                                                    cv::Mat& yellow_binary,
                                                    cv::Mat& white_binary) const {
    if (!yellow_filtered.empty()) {
        cv::Mat yellow_gray;
        cv::cvtColor(yellow_filtered, yellow_gray, cv::COLOR_BGR2GRAY);
        cv::threshold(yellow_gray, yellow_binary, 50.0, 255.0, cv::THRESH_BINARY);
        // 노란색 잡음 제거를 위해 3x3 커널로 열기 연산
        cv::morphologyEx(yellow_binary,
                         yellow_binary,
                         cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    } else {
        yellow_binary.release();
    }

    if (!white_filtered.empty()) {
        cv::Mat white_gray;
        cv::cvtColor(white_filtered, white_gray, cv::COLOR_BGR2GRAY);
        cv::threshold(white_gray, white_binary, 50.0, 255.0, cv::THRESH_BINARY);
        // 흰색 차선도 동일한 방식으로 노이즈를 깎아 후단 알고리즘에 전달
        cv::morphologyEx(white_binary,
                         white_binary,
                         cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    } else {
        white_binary.release();
    }
}

}  // namespace lane_detection
