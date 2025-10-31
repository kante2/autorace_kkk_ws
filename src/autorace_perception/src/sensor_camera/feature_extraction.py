#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import cv2 
import numpy as np

class LaneFeatureExtractor:
    """라인 인식 결과에서 주행 차선/정지선/로터리 정보를 추출하는 모듈."""
    def __init__(self):
        self.init_current_lane()

    def init_current_lane(self):
        """초기 차선 상태를 오른쪽 차선으로 설정."""
        self.current_lane     = "right"  

    def estimate_stop_line(self, white_bin_img, img_y , threshold=240):
        """흰색 바이너리 영상에서 정지선 위치를 찾고 해당 영역을 제거한다."""
        # y축 방향으로 sum → 수평선은 y축 기준으로 sum값이 커짐
        self.img_y = img_y
        horizontal_sum = np.sum(white_bin_img, axis=1) // 255  # shape: (height,)
        stop_line_indices = np.where(horizontal_sum > threshold)[0]
        # threshold 이상인 위치 찾기
        if len(stop_line_indices) > 0:
            min_y = stop_line_indices[0]
            max_y = stop_line_indices[-1]
            
            # 디텍팅된 구간을 검은색(0)으로 만듦
            white_bin_img[min_y:max_y+1, :] = 0

            return [int(min_y), int(max_y)], white_bin_img
        else:
            return [], white_bin_img

    def estimate_lane_mode(self, warped_img):
        """노란/흰 차선의 위치 비중을 통해 현재 주행 차선(좌/우)을 추정."""
        img_hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)

        # HSV 범위로 노란색/흰색 차선을 분리
        yellow_lower = np.array([18, 140, 120]) 
        yellow_upper = np.array([34,255,255])
        yellow_mask  = cv2.inRange(img_hsv, yellow_lower, yellow_upper)

        white_lower = np.array([0,0,192])
        white_upper = np.array([179,64,255])
        white_mask  = cv2.inRange(img_hsv, white_lower, white_upper)

        # 좌우 영역을 각각 분리해 카운트를 비교
        height, width = yellow_mask.shape
        left_roi  = yellow_mask[:, :width//2]
        right_roi = white_mask[:,  width//2:]

        left_yellow_count  = cv2.countNonZero(left_roi)
        right_white_count  = cv2.countNonZero(right_roi)

        #print(f"🟨 left yellow: {left_yellow_count}, ⬜ right white: {right_white_count}")
        # cv2.imshow("Yellow Mask", yellow_mask)
        # cv2.imshow("White Mask", white_mask)
        # cv2.waitKey(1)

        #threshold = 100
        threshold = 9000 # 100 -> 200,, 왼쪽 끝에 노란색이 걸려서 1차선으로 판단하는 경우가 있었음, 
        # print(f"self.current_lane {self.current_lane}")
        # print(f"left_yellow_count {left_yellow_count}  right_white_count {right_white_count} ")
        if left_yellow_count > threshold:
            self.current_lane = "left"
            print(f" left_yellow_count  {left_yellow_count}")
            return "left"
        elif right_white_count > threshold:
            self.current_lane = "right"
            return "right"
        else:
            return self.current_lane  # 변화 없으면 유지
        
    def detect_out_rotary(self,yellow_bin_img):
        """노란 차선이 우측 상단 ROI에 사라졌는지 확인해 로터리 탈출 여부 판단."""
        img = yellow_bin_img
        h, w = img.shape[:2]
        # 로타리 전용 ROI (필요 시 비율 조정)
        x1, x2 = int(0.68 * w), w
        y1, y2 = 0, int(0.42* h )
        roi = img[y1:y2, x1:x2]
        yellow_count = int(cv2.countNonZero(roi))
        return yellow_count > 0

# ---------------------------------------------------------------------------
# 전체 알고리즘과 역할
# - 흰색 바이너리 영상의 수평 화소 합을 이용해 정지선 구간을 탐지하고, 후속 처리를 위해 해당 영역을 제거한다.
# - BEV 이미지에서 노란/흰 차선의 분포를 비교해 현재 주행 차로(좌/우)를 지속적으로 추적한다.
# - 로터리 전용 ROI에서 노란 차선이 관측되는지 확인하여 회전 교차로 탈출 여부를 판별한다.
