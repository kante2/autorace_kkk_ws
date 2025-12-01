#!/usr/bin/env python3
import sys
import cv2
import numpy as np

# -------------------- 파라미터 --------------------
# H: 10~20 주황,  20~30 노랑,  30~40 노랑+연두
# S: 0~30: 무채색, 30~80 누런 색, 80~150: 진한색, 150~255: 쨍한색
# V: 0~50: 매우 어두움;그림자 // 50~100: 어두운 색 // 100~180: 보통 밝기 // 180~255: 아주 밝음

# HSV 범위 디버깅용 코드

g_yellow_lower_1 = (15, 100, 80)    # H, S, V
g_yellow_upper_1 = (40, 255, 255)

g_yellow_lower_2 = (18, 100, 110)
g_yellow_upper_2 = (38, 255, 230)


def binarize_lanes(bgr, lower, upper):
    """주어진 HSV 구간(lower, upper)으로 노란 차선 이진화"""
    # 노이즈 조금 줄이려고 블러
    bgr_blur = cv2.GaussianBlur(bgr, (5, 5), 0)
    hsv = cv2.cvtColor(bgr_blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    # 작은 노이즈 제거용 morphology
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    return mask


def put_label(img, text):
    out = img.copy()
    cv2.putText(out, text, (20, 60),          # 위치
                cv2.FONT_HERSHEY_SIMPLEX,
                3,                          # 글씨 크기 크게
                (255, 255, 255),
                5,                            # 두께
                cv2.LINE_AA)
    return out

def main():
    # 이미지 경로 인자 받기
    img_path = "/root/autorace_kkk_ws/src/perception_node/src/camera/gate_0.png"
    if len(sys.argv) > 1:
        img_path = sys.argv[1]

    bgr = cv2.imread(img_path, cv2.IMREAD_COLOR)
    if bgr is None:
        print(f"failed to load image: {img_path}")
        sys.exit(1)

    h, w = bgr.shape[:2]

    # === ROI 설정 ===
    # 가로: 20% ~ 80% (중앙 60%)
    x0 = int(0.2 * w)
    x1 = int(0.8 * w)
    # 세로: 위쪽 50% (0 ~ 50%)
    y0 = 0
    y1 = int(0.5 * h)

    roi_w = x1 - x0
    roi_h = y1 - y0
    total_pixels = roi_w * roi_h   # ROI 안 픽셀 수

    # --- 각 HSV 범위별로 전방 이미지 이진화 ---
    ranges = [
        ("#1 ", g_yellow_lower_1, g_yellow_upper_1),
        ("#2 ", g_yellow_lower_2, g_yellow_upper_2),
    ]

    mask_imgs = []
    combined_mask_full = np.zeros((h, w), dtype=np.uint8)

    for label, lower, upper in ranges:
        # 전체 프레임에서 이진화
        mask_full = binarize_lanes(bgr, lower, upper)

        # ROI 부분만 사용해서 픽셀 수/퍼센트 계산
        roi_part = mask_full[y0:y1, x0:x1]
        nonzero = int(np.count_nonzero(roi_part))
        ratio = (nonzero / float(total_pixels)) * 100.0
        print(f"{label}: nonzero pixels (ROI) = {nonzero} ({ratio:.2f} %)")

        # 시각화용: ROI 바깥은 0, ROI 안만 살린 마스크
        mask_roi_full = np.zeros_like(mask_full)
        mask_roi_full[y0:y1, x0:x1] = roi_part

        # 전체 OR 마스크 갱신
        combined_mask_full = cv2.bitwise_or(combined_mask_full, mask_roi_full)

        # 3채널로 변환 + 라벨
        mask_bgr = cv2.cvtColor(mask_roi_full, cv2.COLOR_GRAY2BGR)
        mask_labeled = put_label(mask_bgr, f"{label}{ratio:.2f}%")
        mask_imgs.append(mask_labeled)

    # COMBINED 비율도 ROI 기준으로 계산
    combined_roi = combined_mask_full[y0:y1, x0:x1]
    combined_nonzero = int(np.count_nonzero(combined_roi))
    combined_ratio = (combined_nonzero / float(total_pixels)) * 100.0
    print(f"COMBINED (ROI): nonzero pixels = {combined_nonzero} ({combined_ratio:.2f} %)")

    # ------------ 시각화 ------------

    # 1) 위: 원본 전체 프레임 + ROI 빨간 박스(가로 중앙 60%, 세로 위 50%)
    bgr_vis = bgr.copy()
    cv2.rectangle(bgr_vis, (x0, y0), (x1 - 1, y1 - 1), (0, 0, 255), 3)
    top_row = put_label(bgr_vis,
                        f"Original BGR (ROI yellow: {combined_ratio:.2f}%)")

    # 2) 아래: 각 마스크(ROI만 흰색, 나머지는 검정)
    mask_resized = [cv2.resize(m, (w, h)) for m in mask_imgs]
    bottom_row = cv2.hconcat(mask_resized)

    # 가로 길이 맞추기 (필요시 패딩)
    h_top, w_top = top_row.shape[:2]
    h_bot, w_bot = bottom_row.shape[:2]

    #if w_bot < w_top:
    #    pad = np.zeros((h_bot, w_top - w_bot, 3), dtype=bottom_row.dtype)
    #    bottom_row = cv2.hconcat([bottom_row, pad])
    #elif w_bot > w_top:
    #    pad = np.zeros((h_top, w_bot - w_top, 3), dtype=top_row.dtype)
    #    top_row = cv2.hconcat([top_row, pad])

    #canvas_all = cv2.vconcat([top_row, bottom_row])

    #cv2.namedWindow("front_binary", cv2.WINDOW_NORMAL)
    #cv2.imshow("front_binary", canvas_all)
    #cv2.waitKey(0)


if __name__ == "__main__":
    main()
