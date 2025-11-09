#pragma once

#include <opencv2/core.hpp>

namespace lane_detection {

/**
 * @brief 카메라 전처리: BEV 변환, HSV 색상 분리, 이진화.
 *
 * Python `preprocessing.py`의 `CameraPreprocessor` 구현을 C++로 포팅했다.
 */
class CameraPreprocessor {
public:
    CameraPreprocessor();

    /// 일반 주행 환경을 위한 Bird's-Eye View 투시 변환.
    cv::Mat bevWarp(const cv::Mat& img, int img_y, int img_x) const;

    /// 로터리 전용 Bird's-Eye View 투시 변환.
    cv::Mat bevWarpRotary(const cv::Mat& img, int img_y, int img_x) const;

    /// HSV 색상 범위 기반 노란/흰 차선 영역 필터링.
    void detectColorYellowAndWhite(const cv::Mat& img_bgr,
                                   const cv::Mat& img_hsv,
                                   cv::Mat& yellow_filtered,
                                   cv::Mat& white_filtered) const;

    /// 필터링된 BGR 이미지를 바이너리 마스크로 변환.
    void binaryImagesYellowAndWhite(const cv::Mat& yellow_filtered,
                                    const cv::Mat& white_filtered,
                                    cv::Mat& yellow_binary,
                                    cv::Mat& white_binary) const;

private:
    void initColor();

    cv::Scalar yellow_lower_;
    cv::Scalar yellow_upper_;
    cv::Scalar white_lower_;
    cv::Scalar white_upper_;
};

}  // namespace lane_detection
