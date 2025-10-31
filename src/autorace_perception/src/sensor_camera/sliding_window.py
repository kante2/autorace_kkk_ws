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

class SlidingWindow:
    # 슬라이딩 윈도우 방식으로 이진 차선 영상에서 좌우 차선 궤적을 추적
    def __init__(self):
        self.init_sliding()
    def init_sliding(self):
        # 기본 상태 변수 초기화 (이미지 설정 전 초기값 준비)
        self.midpoint = None
        self.is_left_lane = True
        self.img_y = 0
        
        # 탐색에 필요한 창(윈도우) 설정값 및 누적 버퍼
        self.window_height = 0
        self.nwindows = 10
        self.nonzero = None
        self.nonzeroy = None
        self.nonzerox = None
        self.left_blocked_flag = 0
        self.right_blocked_flag = 0
        self.alpha = 0.8  # 추정 x 좌표를 부드럽게 하기 위한 지수 평균 계수
        
        # 각 차선의 시작/끝 추정치
        self.left_lane_start = None
        self.left_lane_end = None
        self.right_lane_start = None
        self.right_lane_end = None
        
        # 각 윈도우에서 찾은 차선 좌표 누적
        self.left_lanes =[]
        self.right_lanes =[]

        # 차선이 서로 겹치지 않도록 하기 위한 최소 간격 설정
        self.stop_flag = False
        self.min_sep = 60

    def set_img_y(self,y):
        self.img_y = y

    def sliding_window_lane_calculation(self, x_current, y_current, prev_margin, x_prev,
                                        blocked_flag, binary_img, is_left_lane,
                                        other_x=None, min_sep=0, color=(0, 255, 0)):
        # 단일 윈도우를 이동시키며 유효 픽셀을 찾고 다음 윈도우 위치를 예측
        flag = blocked_flag

        win_y_low = y_current - self.window_height // 2
        win_y_high = y_current + self.window_height // 2
        win_x_low = x_current - self.margin - prev_margin
        win_x_high = x_current + self.margin + prev_margin

        # 현재 윈도우 위치를 표시
        cv2.rectangle(binary_img, (win_x_low, win_y_low), (win_x_high, win_y_high), color, 2)

        good_inds = ((self.nonzeroy >= win_y_low) & (self.nonzeroy < win_y_high) &
                     (self.nonzerox >= win_x_low) & (self.nonzerox < win_x_high)).nonzero()[0]

        if len(good_inds) > self.minpix:
            min_x = np.min(self.nonzerox[good_inds])
            max_x = np.max(self.nonzerox[good_inds])

            if (max_x - min_x) > 100:
                y_current -= self.window_height
                flag += 1
                return x_current, y_current, flag, self.margin, x_prev

            center_y = y_current
            mean_x = np.mean(self.nonzerox[good_inds])
            delta = mean_x + (mean_x - x_prev)
            x_current = int(self.alpha * mean_x + (1 - self.alpha) * delta)
            y_current = center_y - self.window_height
            prev_margin = min((max_x - min_x) // 4, 20)
            x_prev = x_current

            # 반대 트랙의 현재 x와 너무 가까우면 이 윈도우는 스킵하여 소유권 충돌 방지
            if other_x is not None and abs(x_current - other_x) < min_sep:
                flag += 1
                return x_current, y_current, flag, prev_margin, x_prev

            # 추정된 차선 중심을 표시
            cv2.circle(binary_img, (x_current, y_current), 5, (0, 0, 255), -1)
            flag = 0

            # 생성 라벨 고정: 왼쪽으로 시작했으면 끝까지 왼쪽, 오른쪽도 동일
            if is_left_lane:
                self.left_lanes.append((x_current, y_current))
            else:
                self.right_lanes.append((x_current, y_current))

        else:
            y_current -= self.window_height
            flag += 1

        return x_current, y_current, flag, prev_margin, x_prev

    def sliding_window_adaptive(self, binary_img, nwindows=12, margin=80, minpix=100):
        # 주어진 이진 차선 영상에서 좌우 차선을 동시에 추적하며 결과 영상과 좌표를 반환
        binary_img_color = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)

        self.left_blocked_flag = 0
        self.right_blocked_flag = 0
        self.left_lanes = []
        self.right_lanes = []

        self.margin = margin
        self.minpix = minpix
        self.left_lane_start = 160   # 차선 탐색 시작 x 위치(차선 폭에 맞춰 경험적으로 설정)
        self.right_lane_start = 480

        self.window_height = binary_img.shape[0] // nwindows
        # 이진 영상에서 0이 아닌 픽셀 좌표를 미리 추출하여 반복 사용
        self.nonzero = binary_img.nonzero()
        self.nonzeroy = np.array(self.nonzero[0])   # 차선 후보 픽셀 한 번에 뽑아놓기
        self.nonzerox = np.array(self.nonzero[1])

        prev_left_margin = margin
        prev_right_margin = margin

        leftx_current = self.left_lane_start
        leftx_prev = self.left_lane_start
        lefty_current = binary_img.shape[0] - self.window_height // 2

        rightx_current = self.right_lane_start
        rightx_prev = self.right_lane_start
        righty_current = binary_img.shape[0] - self.window_height // 2

        for _ in range(nwindows):
            # 왼쪽 트랙 먼저 갱신
            if self.left_blocked_flag < 7:
                leftx_current, lefty_current, self.left_blocked_flag, prev_left_margin, leftx_prev = \
                    self.sliding_window_lane_calculation(
                        leftx_current, lefty_current, prev_left_margin, leftx_prev,
                        self.left_blocked_flag, binary_img_color, True,
                        other_x=rightx_current, min_sep=self.min_sep, color=(0, 255, 0))

            # 오른쪽 트랙 갱신
            if self.right_blocked_flag < 7:
                rightx_current, righty_current, self.right_blocked_flag, prev_right_margin, rightx_prev = \
                    self.sliding_window_lane_calculation(
                        rightx_current, righty_current, prev_right_margin, rightx_prev,
                        self.right_blocked_flag, binary_img_color, False,
                        other_x=leftx_current, min_sep=self.min_sep, color=(255, 0, 0))

        return binary_img_color, self.left_lanes, self.right_lanes

# ---------------------------------------------------------------------------
# 전체 알고리즘 개요
# 1) 상태 초기화: 이미지 높이, 윈도우 개수, 차선 시작점, 최소 픽셀 수 등 추적에 필요한 내부 상태를 초기화한다.
# 2) 입력 준비: 이진 차선 영상에서 값이 있는 모든 픽셀 좌표(nonzero)를 미리 추출해 탐색 효율을 높인다.
# 3) 윈도우 설정: 화면을 일정한 높이의 여러 윈도우로 나누고 좌/우 차선 시작 x 좌표를 기준으로 탐색을 시작한다.
# 4) 픽셀 검색: 각 윈도우에서 조건에 맞는 픽셀 인덱스를 찾고 평균 x 위치와 폭을 바탕으로 차선 중심을 추정한다.
# 5) 추정 보정: 이전 프레임의 위치와 지수 평균(alpha)을 사용해 x 좌표를 부드럽게 갱신하고, 과도한 움직임을 제한한다.
# 6) 간섭 방지: 반대 차선과 간격이 너무 좁으면 해당 윈도우를 건너뛰어 좌/우 차선이 뒤섞이지 않도록 한다.
# 7) 누적 저장: 각 층에서 찾은 차선 중심 좌표를 좌/우 리스트에 누적하여 최종 궤적을 구성한다.
# 8) 결과 반환: 시각화를 위한 컬러 바이너리 이미지와 좌/우 차선 좌표 리스트를 반환하여 상위 모듈에서 활용하게 한다.
