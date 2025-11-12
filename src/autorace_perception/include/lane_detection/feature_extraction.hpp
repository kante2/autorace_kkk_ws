#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace lane_detection {

/**
 * @brief 노란 차선 기반 고수준 특징(차선 모드, 정지선, 로터리)을 추정한다.
 *
 * Python `feature_extraction.py`의 `LaneFeatureExtractor`를 C++로 포팅한 구현.
 * 대회 환경이 노란 차선만 존재한다는 전제에 맞춰 오른쪽 차선도 노란색으로 처리한다.
 */
class LaneFeatureExtractor {
public:
    LaneFeatureExtractor();

    /**
     * @brief 정지선 후보 영역을 탐색하고 제거한다.
     *
     * @param binary    정지선 탐색용 이진 영상(차선 색상과 무관)
     * @param img_y     원본 이미지 세로 길이
     * @param threshold 정지선으로 판단할 최소 픽셀 수
     * @return 정지선이 있으면 [min_y, max_y], 없으면 빈 벡터
     */
    std::vector<int> estimateStopLine(cv::Mat& binary, int img_y, int threshold = 240) const;

    /**
     * @brief BEV BGR 영상에서 노란 차선의 분포를 비교하여 현재 차로(좌/우)를 추정한다.
     * @return "left" 또는 "right"
     */
    std::string estimateLaneMode(const cv::Mat& bev_bgr);

    /**
     * @brief 노란 차선이 특정 ROI에서 사라졌는지 여부로 로터리 탈출을 판단한다.
     */
    bool detectOutRotary(const cv::Mat& yellow_binary) const;

    /**
     * @brief 직전 차로 상태 반환.
     */
    const std::string& currentLane() const { return current_lane_; }

private:
    void initCurrentLane();

    std::string current_lane_;
};

}  // namespace lane_detection