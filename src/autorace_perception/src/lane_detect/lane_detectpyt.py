#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import json
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class LaneDetectorNode:
    def __init__(self):
        rospy.init_node("lane_detector_node")
        rospy.loginfo("lane_detector_node started")

        # === OpenCV windows ===
        self.show_window = rospy.get_param("~show_window", True)  # 기본 True
        self.win_name_main = "lane_debug"
        self.win_name_bev  = "lane_bev"
        self.win_name_bev_raw = "lane_bev_raw"   # BEV로 워프된 원본(BGR)
        self.win_name_bev_bin = "lane_bev_bin"   # BEV에서 이진화만 본 화면

        if self.show_window:
            try:
                cv2.startWindowThread()
            except Exception:
                pass
            cv2.namedWindow(self.win_name_main, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.win_name_main, 960, 540)

            cv2.namedWindow(self.win_name_bev, cv2.WINDOW_NORMAL)      # 기존: 슬윈 디버그
            cv2.resizeWindow(self.win_name_bev, 640, 480)

            # ★ 새 창들
            cv2.namedWindow(self.win_name_bev_raw, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.win_name_bev_raw, 640, 480)
            cv2.namedWindow(self.win_name_bev_bin, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.win_name_bev_bin, 640, 480)

        rospy.on_shutdown(self._on_shutdown)

        # === Bridge ===
        self.bridge = CvBridge()

        # === Topics ===
        self.sub = rospy.Subscriber(
            "/usb_cam/image_rect_color",
            Image,
            self.CB_cam_raw,
            queue_size=3,
            buff_size=2**24
        )
        self.pub = rospy.Publisher("/perception/camera", String, queue_size=10, latch=False)
        self.pub_debug = rospy.Publisher("/perception/camera_debug", Image, queue_size=1)

        # === Sliding-window Parameters ===
        self.nwindows = rospy.get_param("~nwindows", 12)
        self.margin   = rospy.get_param("~margin", 80)
        self.minpix   = rospy.get_param("~minpix", 50)
        self.min_sep  = rospy.get_param("~min_sep", 60)
        self.alpha    = rospy.get_param("~alpha", 0.8)
        self.debug    = rospy.get_param("~debug", True)

        # === Curvature scale (optional, for meter unit) ===
        # BEV에서 1픽셀당 몇 미터인지 (알면 넣고, 모르면 0으로 두면 m단위 계산 건너뜀)
        self.xm_per_pix = rospy.get_param("~xm_per_pix", 0.0)   # ex) 0.01  (1px = 1cm)
        self.ym_per_pix = rospy.get_param("~ym_per_pix", 0.0)

        # === ROI ratios (원하면 런타임 튜닝) ===
        self.roi_top_y_ratio     = rospy.get_param("~roi_top_y_ratio",     0.50)
        self.roi_left_top_ratio  = rospy.get_param("~roi_left_top_ratio",  0.10)
        self.roi_right_top_ratio = rospy.get_param("~roi_right_top_ratio", 0.90)
        self.roi_left_bot_ratio  = rospy.get_param("~roi_left_bot_ratio",  -0.10)
        self.roi_right_bot_ratio = rospy.get_param("~roi_right_bot_ratio", 1.10)
        self.roi_left_bot_px_push = rospy.get_param("~roi_left_bot_px_push", 20)  # 왼쪽 바닥 더 확장

        # 색 이진화(필요시 튜닝)
        # Yellow
        self.yellow_lower = np.array([10,  80,  60], dtype=np.uint8)
        self.yellow_upper = np.array([45, 255, 255], dtype=np.uint8)

        # White
        self.white_lower  = np.array([ 0,   0, 150], dtype=np.uint8)
        self.white_upper  = np.array([179,  60, 255], dtype=np.uint8)


        self._last_hb = rospy.Time(0)

    # ---------- encoders ----------
    def encode_image(self, bgr):
        msg = self.bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera"
        return msg

    def encode_compressed(self, bgr, quality=80):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
        ok, enc = cv2.imencode(".jpg", bgr, encode_param)
        if not ok:
            return None
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = enc.tobytes()
        return msg

    # --------- 이진화 ---------
    def to_binary(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask_y = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        mask_w = cv2.inRange(hsv, self.white_lower,  self.white_upper)
        kernel = np.ones((3,3), np.uint8)
        mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_OPEN, kernel, iterations=1)
        mask_w = cv2.morphologyEx(mask_w, cv2.MORPH_OPEN, kernel, iterations=1)
        return cv2.bitwise_or(mask_y, mask_w)

    # --------- ROI ----------
    def make_roi_polygon(self, h, w):
        y_top = int(h * self.roi_top_y_ratio)
        y_bot = h - 1
        x_left_top  = int(w * self.roi_left_top_ratio)
        x_right_top = int(w * self.roi_right_top_ratio)
        x_left_bot  = int(w * self.roi_left_bot_ratio)
        x_right_bot = int(w * self.roi_right_bot_ratio)
        #x_left_bot  = max(0, x_left_bot - int(self.roi_left_bot_px_push))  # 음수 방지 클램프
        return np.array([[x_left_bot,  y_bot],     # BL
                         [x_left_top,  y_top],     # TL
                         [x_right_top, y_top],     # TR
                         [x_right_bot, y_bot]],    # BR
                        dtype=np.int32)

    def apply_roi(self, binary, roi_poly):
        mask = np.zeros_like(binary, dtype=np.uint8)
        cv2.fillPoly(mask, [roi_poly], 255)
        return cv2.bitwise_and(binary, mask), mask

    def overlay_roi_on_bgr(self, bgr, roi_poly, color=(0,255,0), alpha=0.25, edge_thickness=2):
        overlay = bgr.copy()
        cv2.fillPoly(overlay, [roi_poly], color)
        out = cv2.addWeighted(overlay, alpha, bgr, 1 - alpha, 0)
        cv2.polylines(out, [roi_poly], True, (0,0,0), edge_thickness)
        return out

    # ---------- BEV (Perspective Warp) ----------
    """def bev_warp(self, bgr, roi_poly, dst_size=None):
        
        roi_poly (BL,TL,TR,BR) 사다리꼴을 직사각형으로 폄.
        dst_size = (W,H). 생략 시 W=입력폭, H=roi 높이.
        
        h, w = bgr.shape[:2]
        BL, TL, TR, BR = roi_poly.astype(np.float32)

        if dst_size is None:
            roi_h = int(max(1, BR[1] - TL[1]))
            dst_w = w
            dst_h = roi_h
        else:
            dst_w, dst_h = dst_size

        dst = np.float32([
            [0,          dst_h-1],  # BL -> (0, H-1)
            [0,          0],        # TL -> (0, 0)
            [dst_w-1,    0],        # TR -> (W-1, 0)
            [dst_w-1,    dst_h-1],  # BR -> (W-1, H-1)
        ])

        src = np.float32([BL, TL, TR, BR])
        M = cv2.getPerspectiveTransform(src, dst)
        bev = cv2.warpPerspective(
            bgr, M, (dst_w, dst_h),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REPLICATE
        )
        return bev, M, (dst_w, dst_h)"""

    def bev_warp(self, bgr, roi_poly, dst_size=None):
        """
        roi_poly (BL,TL,TR,BR) 사다리꼴을 직사각형으로 펴되,
        ROI가 이미지 밖으로 벗어나면 해당 부분은 검은색으로 처리.
        """
        h, w = bgr.shape[:2]
        BL, TL, TR, BR = roi_poly.astype(np.float32)

        # === 1) 이미지 밖으로 벗어난 좌표를 클램프 ===
        roi_poly_clamped = np.copy(roi_poly)
        roi_poly_clamped[:, 0] = np.clip(roi_poly[:, 0], 0, w - 1)
        roi_poly_clamped[:, 1] = np.clip(roi_poly[:, 1], 0, h - 1)
        BL, TL, TR, BR = roi_poly_clamped.astype(np.float32)

        # === 2) 출력 크기 설정 ===
        if dst_size is None:
            roi_h = int(max(1, BR[1] - TL[1]))
            dst_w = w
            dst_h = roi_h
        else:
            dst_w, dst_h = dst_size

        dst = np.float32([
            [0,          dst_h - 1],  # BL -> (0, H-1)
            [0,          0],          # TL -> (0, 0)
            [dst_w - 1,  0],          # TR -> (W-1, 0)
            [dst_w - 1,  dst_h - 1],  # BR -> (W-1, H-1)
        ])

        src = np.float32([BL, TL, TR, BR])
        M = cv2.getPerspectiveTransform(src, dst)

        # === 3) warpPerspective with black fill ===
        bev = cv2.warpPerspective(
            bgr, M, (dst_w, dst_h),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0)  # 검정색 채움
        )

        return bev, M, (dst_w, dst_h)


    # --------- 슬라이딩 윈도우 (중심점/곡률 추가됨) ---------
    def sliding_window(self, binary_mask):
        # --- 준비: 비영 픽셀 좌표/크기 ---
        nonzero = binary_mask.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        h, w = binary_mask.shape[:2]

        # --- 하단 히스토그램으로 초기 좌/우 베이스 ---
        histogram = np.sum(binary_mask[h//2:,:], axis=0)
        midpoint = w // 2
        leftx_base  = np.argmax(histogram[:midpoint]) if histogram[:midpoint].any() else None
        rightx_base = (np.argmax(histogram[midpoint:]) + midpoint) if histogram[midpoint:].any() else None

        # --- 디버그 캔버스/윈도우 높이 ---
        out_img = cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2BGR)
        window_height = np.int32(h / self.nwindows)

        leftx_current  = leftx_base
        rightx_current = rightx_base

        left_lane_inds = []
        right_lane_inds = []
        win_color = (255, 0, 0)

        # 추가: 층별 중심점 저장(곡률용)
        left_centers = []   # [(yc, xc), ...]
        right_centers = []  # 필요하면 우측도

        # --- 창 스캔 루프 ---
        for win in range(self.nwindows):
            win_y_low  = h - (win+1) * window_height
            win_y_high = h - win * window_height

            if leftx_current is not None:
                lx1 = leftx_current - self.margin; lx2 = leftx_current + self.margin
                cv2.rectangle(out_img, (lx1, win_y_low), (lx2, win_y_high), win_color, 2)
            if rightx_current is not None:
                rx1 = rightx_current - self.margin; rx2 = rightx_current + self.margin
                cv2.rectangle(out_img, (rx1, win_y_low), (rx2, win_y_high), win_color, 2)

            good_left_inds = []
            good_right_inds = []
            if leftx_current is not None:
                good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                  (nonzerox >= leftx_current - self.margin) &
                                  (nonzerox <  leftx_current + self.margin)).nonzero()[0].tolist()
            if rightx_current is not None:
                good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                   (nonzerox >= rightx_current - self.margin) &
                                   (nonzerox <  rightx_current + self.margin)).nonzero()[0].tolist()

            # 최소 분리 보장
            if leftx_current is not None and rightx_current is not None:
                if abs(leftx_current - rightx_current) < self.min_sep:
                    if len(good_left_inds) < len(good_right_inds):
                        good_left_inds = []
                    else:
                        good_right_inds = []

            # 인덱스 누적
            left_lane_inds.extend(good_left_inds)
            right_lane_inds.extend(good_right_inds)

            # === (추가) 층별 중심점 찍기 & 저장 ===
            yc = (win_y_low + win_y_high) // 2
            if len(good_left_inds) > 0:
                x_mean_left = float(np.mean(nonzerox[good_left_inds]))
                cv2.circle(out_img, (int(x_mean_left), int(yc)), 4, (0,0,255), -1)  # 빨간 점
                left_centers.append((int(yc), float(x_mean_left)))
            if len(good_right_inds) > 0:
                x_mean_right = float(np.mean(nonzerox[good_right_inds]))
                cv2.circle(out_img, (int(x_mean_right), int(yc)), 4, (0,255,255), -1)  # 노랑 점
                right_centers.append((int(yc), float(x_mean_right)))

            # 충분하면 창 중심 EMA 재조정
            if len(good_left_inds) > self.minpix and leftx_current is not None:
                x_mean = float(np.mean(nonzerox[good_left_inds]))
                leftx_current = int(self.alpha * leftx_current + (1 - self.alpha) * x_mean)
            if len(good_right_inds) > self.minpix and rightx_current is not None:
                x_mean = float(np.mean(nonzerox[good_right_inds]))
                rightx_current = int(self.alpha * rightx_current + (1 - self.alpha) * x_mean)

        # --- 픽셀 좌표 집계 ---
        leftx = nonzerox[left_lane_inds] if left_lane_inds else np.array([])
        lefty = nonzeroy[left_lane_inds] if left_lane_inds else np.array([])
        rightx = nonzerox[right_lane_inds] if right_lane_inds else np.array([])
        righty = nonzeroy[right_lane_inds] if right_lane_inds else np.array([])

        # 정수 캐스팅/클리핑
        if leftx.size and lefty.size:
            leftx = np.clip(leftx.astype(np.intp), 0, w-1)
            lefty = np.clip(lefty.astype(np.intp), 0, h-1)
        if rightx.size and righty.size:
            rightx = np.clip(rightx.astype(np.intp), 0, w-1)
            righty = np.clip(righty.astype(np.intp), 0, h-1)

        # 디버그 포인트(픽셀 집합)
        if leftx.size and lefty.size:
            out_img[lefty, leftx] = (0, 0, 255)
        if rightx.size and righty.size:
            out_img[righty, rightx] = (0, 255, 0)

        # --- 2차 다항 피팅(픽셀 전체) ---
        left_fit = None
        right_fit = None
        if leftx.size >= 20:
            left_fit = np.polyfit(lefty.astype(np.float64), leftx.astype(np.float64), 2)
        if rightx.size >= 20:
            right_fit = np.polyfit(righty.astype(np.float64), rightx.astype(np.float64), 2)

        # --- (추가) 중심점 기반 곡률(픽셀 단위) ---
        left_curvature_px = None
        right_curvature_px = None
        if len(left_centers) >= 5:
            ys = np.array([p[0] for p in left_centers], dtype=np.float64)
            xs = np.array([p[1] for p in left_centers], dtype=np.float64)
            a,b,c = np.polyfit(ys, xs, 2)   # x = a*y^2 + b*y + c
            y0 = float(h - 1)               # 하단부에서의 곡률
            dxdy  = 2*a*y0 + b
            d2xdy2 = 2*a
            left_curvature_px = abs(d2xdy2) / ((1.0 + dxdy*dxdy) ** 1.5)  # [1/px]
        if len(right_centers) >= 5:
            ys = np.array([p[0] for p in right_centers], dtype=np.float64)
            xs = np.array([p[1] for p in right_centers], dtype=np.float64)
            a,b,c = np.polyfit(ys, xs, 2)
            y0 = float(h - 1)
            dxdy  = 2*a*y0 + b
            d2xdy2 = 2*a
            right_curvature_px = abs(d2xdy2) / ((1.0 + dxdy*dxdy) ** 1.5)

        # --- (옵션) 미터 단위 곡률/반경 ---
        left_curvature_m = None
        right_curvature_m = None
        left_radius_m = None
        right_radius_m = None
        if self.xm_per_pix > 0 and self.ym_per_pix > 0:
            if len(left_centers) >= 5:
                ys_m = np.array([p[0] for p in left_centers], dtype=np.float64) * self.ym_per_pix
                xs_m = np.array([p[1] for p in left_centers], dtype=np.float64) * self.xm_per_pix
                a,b,c = np.polyfit(ys_m, xs_m, 2)
                y0m = (h - 1) * self.ym_per_pix
                dxdy  = 2*a*y0m + b
                d2xdy2 = 2*a
                kappa = abs(d2xdy2) / ((1.0 + dxdy*dxdy) ** 1.5)  # [1/m]
                left_curvature_m = kappa
                left_radius_m = (1.0 / kappa) if kappa > 1e-9 else None
            if len(right_centers) >= 5:
                ys_m = np.array([p[0] for p in right_centers], dtype=np.float64) * self.ym_per_pix
                xs_m = np.array([p[1] for p in right_centers], dtype=np.float64) * self.xm_per_pix
                a,b,c = np.polyfit(ys_m, xs_m, 2)
                y0m = (h - 1) * self.ym_per_pix
                dxdy  = 2*a*y0m + b
                d2xdy2 = 2*a
                kappa = abs(d2xdy2) / ((1.0 + dxdy*dxdy) ** 1.5)
                right_curvature_m = kappa
                right_radius_m = (1.0 / kappa) if kappa > 1e-9 else None

        # --- 피팅 곡선 점찍기(시각화) ---
        ploty = np.linspace(0, h-1, h).astype(np.int32)
        if left_fit is not None:
            left_fitx = (left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]).astype(np.int32)
            for y, x in zip(ploty[::10], left_fitx[::10]):
                cv2.circle(out_img, (int(x), int(y)), 2, (255,255,0), -1)
        if right_fit is not None:
            right_fitx = (right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]).astype(np.int32)
            for y, x in zip(ploty[::10], right_fitx[::10]):
                cv2.circle(out_img, (int(x), int(y)), 2, (0,255,255), -1)

        # --- 결과 ---
        return {
            "left_fit": left_fit.tolist() if left_fit is not None else None,
            "right_fit": right_fit.tolist() if right_fit is not None else None,
            "debug_img": out_img,
            # 추가 정보
            "left_centers": left_centers,            # [(y,x), ...]
            "right_centers": right_centers,
            "left_curvature_px": left_curvature_px,  # [1/px] or None
            "right_curvature_px": right_curvature_px,
            "left_curvature_m": left_curvature_m,    # [1/m] or None
            "right_curvature_m": right_curvature_m,
            "left_radius_m": left_radius_m,          # [m] or None
            "right_radius_m": right_radius_m
        }

    # ---------- 종료 훅 ----------
    def _on_shutdown(self):
        if self.show_window:
            try:
                cv2.destroyWindow(self.win_name_main)
                cv2.destroyWindow(self.win_name_bev)
            except Exception:
                pass

    # ---------- 콜백 ----------
    def CB_cam_raw(self, msg):
        try:
            # heartbeat
            now = rospy.Time.now()
            if (now - self._last_hb).to_sec() > 1.0:
                hb = {"heartbeat": True, "t": now.to_sec()}
                self.pub.publish(String(data=json.dumps(hb)))
                rospy.loginfo_throttle(1.0, "[lane_detector_node] heartbeat publish")
                self._last_hb = now

            # Image -> BGR
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if bgr is None:
                rospy.logwarn_throttle(2.0, "[lane_detector_node] empty frame")
                return
            if len(bgr.shape) == 2 or (len(bgr.shape) == 3 and bgr.shape[2] == 1):
                bgr = cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR)

            h, w = bgr.shape[:2]

            # 1) ROI 폴리곤
            roi_poly = self.make_roi_polygon(h, w)

            # 2) 원본 위 ROI 오버레이(메인 창)
            vis_main = self.overlay_roi_on_bgr(bgr, roi_poly, color=(0,255,0), alpha=0.25, edge_thickness=2)

            # 3) BEV: ROI 사다리꼴 -> 직사각형
            bev_bgr, M, (bev_w, bev_h) = self.bev_warp(bgr, roi_poly)  # bgr를 폄

            # 4) BEV에서 이진화 + 슬라이딩 윈도우 (차선 인식은 BEV 기준)
            bev_bin = self.to_binary(bev_bgr)
            bev_sw  = self.sliding_window(bev_bin)
            bev_vis = bev_sw["debug_img"]  # BEV상 윈도우/포인트/피팅 + 빨간 중심점

            # 5) JSON 결과(피팅/곡률은 BEV 좌표계 기준)
            result = {
                "header": {"stamp": rospy.Time.now().to_sec(), "frame_id": "camera"},
                "roi": roi_poly.tolist(),
                "bev_size": [int(bev_w), int(bev_h)],
                "left_fit_bev": bev_sw["left_fit"],       # x = a*y^2 + b*y + c
                "right_fit_bev": bev_sw["right_fit"],
                "left_curvature_px": bev_sw["left_curvature_px"],
                "right_curvature_px": bev_sw["right_curvature_px"],
                "left_curvature_m": bev_sw["left_curvature_m"],
                "right_curvature_m": bev_sw["right_curvature_m"],
                "left_radius_m": bev_sw["left_radius_m"],
                "right_radius_m": bev_sw["right_radius_m"],
                "params": {
                    "nwindows": self.nwindows, "margin": self.margin,
                    "minpix": self.minpix, "min_sep": self.min_sep,
                    "alpha": self.alpha,
                    "xm_per_pix": self.xm_per_pix, "ym_per_pix": self.ym_per_pix
                }
            }
            self.pub.publish(String(data=json.dumps(result, ensure_ascii=False)))

            # 6) 디버그 이미지 퍼블리시 (메인: 원본오버레이)
            if self.debug:
                self.pub_debug.publish(self.encode_image(vis_main))

            # 7) OpenCV 창 표시
            if self.show_window:
                cv2.imshow(self.win_name_main, vis_main)  # 원본+ROI
                cv2.imshow(self.win_name_bev, bev_vis)    # BEV+슬윈+중심점

                    # ★ 추가: BEV 원본과 BEV 이진화만 별도 창으로
                cv2.imshow(self.win_name_bev_raw, bev_bgr)    # 워프된 컬러
                cv2.imshow(self.win_name_bev_bin, bev_bin)    # 이진(단채널)

                cv2.waitKey(1)

            rospy.loginfo_throttle(1.0, "[lane_detector_node] frame processed & published")

        except CvBridgeError as e:
            rospy.logwarn(f"[lane_detector_node] CvBridgeError: {e}")
        except Exception as e:
            rospy.logwarn(f"[lane_detector_node] exception: {e}")

    def spin(self):
        rospy.loginfo("lane_detector_node is running...")
        rospy.spin()

if __name__ == "__main__":
    node = LaneDetectorNode()
    node.spin()
