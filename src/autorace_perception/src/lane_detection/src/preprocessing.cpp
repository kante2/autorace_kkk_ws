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
    yellow_lower_ = cv::Scalar(15, 128, 0);
    yellow_upper_ = cv::Scalar(40, 255, 255);
}

cv::Mat CameraPreprocessor::bevWarp(const cv::Mat& img, int img_y, int img_x) const {
    constexpr float delta = 25.0f;

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

    // 흰 차선은 사용하지 않으므로 동일 크기의 0 행렬로 반환
    white_filtered = cv::Mat::zeros(img_bgr.size(), img_bgr.type());
}

void CameraPreprocessor::binaryImagesYellowAndWhite(const cv::Mat& yellow_filtered,
                                                    const cv::Mat& white_filtered,
                                                    cv::Mat& yellow_binary,
                                                    cv::Mat& white_binary) const {
    if (!yellow_filtered.empty()) {
        cv::Mat yellow_gray;
        cv::cvtColor(yellow_filtered, yellow_gray, cv::COLOR_BGR2GRAY);
        cv::threshold(yellow_gray, yellow_binary, 50.0, 255.0, cv::THRESH_BINARY);
        // 노이즈 억제를 위해 작은 커널로 열기 연산
        cv::morphologyEx(yellow_binary,
                         yellow_binary,
                         cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    } else {
        yellow_binary.release();
    }

    cv::Size mask_size;
    if (!yellow_filtered.empty()) {
        mask_size = yellow_filtered.size();
    } else if (!white_filtered.empty()) {
        mask_size = white_filtered.size();
    }
    if (mask_size.width > 0 && mask_size.height > 0) {
        white_binary = cv::Mat::zeros(mask_size, CV_8UC1);
    } else {
        white_binary.release();
    }
}

}  // namespace lane_detection
