#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LaneLiteNode:
    def __init__(self):
        rospy.init_node("lane_lite_node")
        rospy.loginfo("lane_lite_node started")

        # --- UI ---
        self.show_window = rospy.get_param("~show_window", True)
        self.win_main = "lane_viz"
        if self.show_window:
            try: cv2.startWindowThread()
            except Exception: pass
            cv2.namedWindow(self.win_main, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.win_main, 960, 540)

        # --- Bridge & IO ---
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/usb_cam/image_rect_color", Image, self.cb, queue_size=2, buff_size=2**24)

        # (선택) 간단 퍼블리시: [left_mean_x, right_mean_x, kappa_left_px, kappa_right_px, kappa_center_px]
        # 안 쓸 거면 주석 처리
        from std_msgs.msg import Float32MultiArray
        self.pub_vals = rospy.Publisher("/perception/lane_brief", Float32MultiArray, queue_size=1)

        # --- 파라미터 (필요 최소만) ---
        self.nwindows = rospy.get_param("~nwindows", 12)
        self.margin   = rospy.get_param("~margin", 80)
        self.minpix   = rospy.get_param("~minpix", 50)
        self.min_sep  = rospy.get_param("~min_sep", 60)
        self.alpha    = rospy.get_param("~alpha", 0.8)

        # ROI 비율
        self.roi_top_y_ratio     = rospy.get_param("~roi_top_y_ratio",     0.60)
        self.roi_left_top_ratio  = rospy.get_param("~roi_left_top_ratio",  0.25)
        self.roi_right_top_ratio = rospy.get_param("~roi_right_top_ratio", 0.75)
        self.roi_left_bot_ratio  = rospy.get_param("~roi_left_bot_ratio",  -0.45)
        self.roi_right_bot_ratio = rospy.get_param("~roi_right_bot_ratio", 1.45)

        # 색 이진화(간단)
        self.yellow_lower = np.array([10,  80,  60], dtype=np.uint8)
        self.yellow_upper = np.array([45, 255, 255], dtype=np.uint8)
        self.white_lower  = np.array([ 0,   0, 150], dtype=np.uint8)
        self.white_upper  = np.array([179,  60, 255], dtype=np.uint8)

        # 곡률(m) 환산(선택): 0이면 px만 사용
        self.xm_per_pix = rospy.get_param("~xm_per_pix", 0.0)
        self.ym_per_pix = rospy.get_param("~ym_per_pix", 0.0)
        self.reference_len_m = rospy.get_param("~reference_len_m", 0.50)
        self._scale_set = False

    # ------------------- core helpers -------------------
    def make_roi_polygon(self, h, w):
        y_top = int(h * self.roi_top_y_ratio); y_bot = h - 1
        x_lt  = int(w * self.roi_left_top_ratio)
        x_rt  = int(w * self.roi_right_top_ratio)
        x_lb  = int(w * self.roi_left_bot_ratio)
        x_rb  = int(w * self.roi_right_bot_ratio)
        return np.array([[x_lb, y_bot],[x_lt, y_top],[x_rt, y_top],[x_rb, y_bot]], np.int32)

    def bev_warp(self, bgr, roi_poly):
        h, w = bgr.shape[:2]
        BL, TL, TR, BR = roi_poly.astype(np.float32)
        BL[1] = np.clip(BL[1], 0, h-1); TL[1] = np.clip(TL[1], 0, h-1)
        TR[1] = np.clip(TR[1], 0, h-1); BR[1] = np.clip(BR[1], 0, h-1)
        src = np.float32([BL, TL, TR, BR])
        dst = np.float32([[0,h-1],[0,0],[w-1,0],[w-1,h-1]])
        M = cv2.getPerspectiveTransform(src, dst)
        bev = cv2.warpPerspective(bgr, M, (w, h), flags=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))
        return bev

    def to_binary(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask_y = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        mask_w = cv2.inRange(hsv, self.white_lower,  self.white_upper)
        kernel = np.ones((3,3), np.uint8)
        mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_OPEN, kernel, iterations=1)
        mask_w = cv2.morphologyEx(mask_w, cv2.MORPH_OPEN, kernel, iterations=1)
        return cv2.bitwise_or(mask_y, mask_w)

    def sliding_window_and_metrics(self, binary_mask):
        """
        반환(딕셔너리 X):
          out_img(BGR 디버그),
          left_mean_x(float or np.nan),
          right_mean_x(float or np.nan),
          kappa_left_px(float or np.nan),
          kappa_right_px(float or np.nan),
          kappa_center_px(float or np.nan),
          kappa_left_m(optional or np.nan), kappa_right_m, kappa_center_m
        """
        nz = binary_mask.nonzero()
        nonzeroy = np.array(nz[0]); nonzerox = np.array(nz[1])
        h, w = binary_mask.shape[:2]

        histogram = np.sum(binary_mask[h//2:,:], axis=0)
        midpoint = w // 2
        leftx_base = np.argmax(histogram[:midpoint]) if histogram[:midpoint].any() else None
        rightx_base = (np.argmax(histogram[midpoint:]) + midpoint) if histogram[midpoint:].any() else None

        out_img = cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2BGR)
        window_height = int(h / self.nwindows)

        leftx_current, rightx_current = leftx_base, rightx_base
        left_inds = []; right_inds = []
        left_centers = []; right_centers = []

        for win in range(self.nwindows):
            wy_low  = h - (win+1) * window_height
            wy_high = h - win * window_height

            if leftx_current is not None:
                lx1 = leftx_current - self.margin; lx2 = leftx_current + self.margin
                cv2.rectangle(out_img, (lx1, wy_low), (lx2, wy_high), (255,0,0), 2)
            if rightx_current is not None:
                rx1 = rightx_current - self.margin; rx2 = rightx_current + self.margin
                cv2.rectangle(out_img, (rx1, wy_low), (rx2, wy_high), (255,0,0), 2)

            goodL = []; goodR = []
            if leftx_current is not None:
                goodL = ((nonzeroy >= wy_low) & (nonzeroy < wy_high) &
                         (nonzerox >= leftx_current - self.margin) &
                         (nonzerox <  leftx_current + self.margin)).nonzero()[0].tolist()
            if rightx_current is not None:
                goodR = ((nonzeroy >= wy_low) & (nonzeroy < wy_high) &
                         (nonzerox >= rightx_current - self.margin) &
                         (nonzerox <  rightx_current + self.margin)).nonzero()[0].tolist()

            # 차선 간 최소 분리
            if leftx_current is not None and rightx_current is not None:
                if abs(leftx_current - rightx_current) < self.min_sep:
                    if len(goodL) < len(goodR): goodL = []
                    else: goodR = []

            left_inds.extend(goodL); right_inds.extend(goodR)

            yc = (wy_low + wy_high)//2
            if len(goodL) > 0:
                xL = float(np.mean(nonzerox[goodL]))
                left_centers.append((yc, xL))
                cv2.circle(out_img, (int(xL), int(yc)), 4, (0,0,255), -1)
            if len(goodR) > 0:
                xR = float(np.mean(nonzerox[goodR]))
                right_centers.append((yc, xR))
                cv2.circle(out_img, (int(xR), int(yc)), 4, (0,255,255), -1)

            # 창 중심 업데이트(EMA)
            if len(goodL) > self.minpix and leftx_current is not None:
                leftx_current = int(self.alpha * leftx_current + (1 - self.alpha) * float(np.mean(nonzerox[goodL])))
            if len(goodR) > self.minpix and rightx_current is not None:
                rightx_current = int(self.alpha * rightx_current + (1 - self.alpha) * float(np.mean(nonzerox[goodR])))

        # 시각화용 포인트
        if len(left_inds) > 0:
            lx = np.clip(nonzerox[left_inds], 0, w-1)
            ly = np.clip(nonzeroy[left_inds], 0, h-1)
            out_img[ly, lx] = (0, 0, 255)
        if len(right_inds) > 0:
            rx = np.clip(nonzerox[right_inds], 0, w-1)
            ry = np.clip(nonzeroy[right_inds], 0, h-1)
            out_img[ry, rx] = (0, 255, 0)

        # 2차 피팅 (x = a y^2 + b y + c)
        left_fit = right_fit = center_fit = None
        if len(left_inds) >= 20:
            left_fit = np.polyfit(nonzeroy[left_inds].astype(np.float64),
                                  nonzerox[left_inds].astype(np.float64), 2)
        if len(right_inds) >= 20:
            right_fit = np.polyfit(nonzeroy[right_inds].astype(np.float64),
                                   nonzerox[right_inds].astype(np.float64), 2)
        if left_fit is not None and right_fit is not None: # 양 차선이 인지된 경우 -> 양쪽 차선의 곡률 평균
            center_fit = 0.5*(left_fit + right_fit)

        # 곡률 kappa(px): |x''| / (1 + x'^2)^(3/2), y=h-1에서 평가
        def kappa_px(fit, y0):
            a,b,_ = fit
            dxdy = 2*a*y0 + b
            d2  = 2*a
            return abs(d2) / ((1.0 + dxdy*dxdy) ** 1.5)

        y_eval = float(h - 1)
        kL = float("nan"); kR = float("nan"); kC = float("nan")
        if left_fit  is not None:  kL = kappa_px(left_fit,  y_eval)
        if right_fit is not None:  kR = kappa_px(right_fit, y_eval)
        if center_fit is not None: kC = kappa_px(center_fit, y_eval)

        # (선택) m단위 곡률: 스케일 존재 시 변환
        kLm = kRm = kCm = float("nan")
        if self.xm_per_pix > 0 and self.ym_per_pix > 0:
            Xm, Ym = self.xm_per_pix, self.ym_per_pix
            def kappa_m(fit):
                a_px, b_px, c_px = fit
                a_m = a_px * (Xm / (Ym*Ym))
                b_m = b_px * (Xm / Ym)
                y0m = (h - 1) * Ym
                dxdy = 2*a_m*y0m + b_m
                d2   = 2*a_m
                return abs(d2) / ((1.0 + dxdy*dxdy) ** 1.5)
            if left_fit  is not None:  kLm = kappa_m(left_fit)
            if right_fit is not None:  kRm = kappa_m(right_fit)
            if center_fit is not None: kCm = kappa_m(center_fit)

        # 좌/우 중심 평균(x)
        left_mean_x  = float(np.mean([x for _,x in left_centers]))  if len(left_centers)>0  else float("nan")
        right_mean_x = float(np.mean([x for _,x in right_centers])) if len(right_centers)>0 else float("nan")

        # 피팅 시각화(점 간격을 넉넉히)
        ploty = np.linspace(0, h-1, h).astype(np.int32)
        def draw_fit(fit, color):
            fitx = (fit[0]*ploty**2 + fit[1]*ploty + fit[2]).astype(np.int32)
            for y,x in zip(ploty[::10], fitx[::10]):
                cv2.circle(out_img, (int(x), int(y)), 2, color, -1)
        if left_fit  is not None:  draw_fit(left_fit,  (255,255,0))
        if right_fit is not None:  draw_fit(right_fit, (0,255,255))
        if center_fit is not None:  draw_fit(center_fit,(255,0,255))

        # 텍스트 오버레이
        def put(txt, y):
            cv2.putText(out_img, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)
        put(f"L_mean_x: {left_mean_x:.1f} | R_mean_x: {right_mean_x:.1f}", 24)
        put(f"kappa_px  L:{kL:.4e}  R:{kR:.4e}  C:{kC:.4e}", 48)
        if not np.isnan(kLm):
            put(f"kappa_m   L:{kLm:.4e}  R:{kRm:.4e}  C:{kCm:.4e}", 72)

        return (out_img, left_mean_x, right_mean_x, kL, kR, kC, kLm, kRm, kCm)

    # ------------------- ROS callback -------------------
    def cb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if bgr is None: return
            if bgr.ndim == 2 or (bgr.ndim==3 and bgr.shape[2]==1):
                bgr = cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR)

            h, w = bgr.shape[:2]

            # ROI → BEV
            roi_poly = self.make_roi_polygon(h, w)
            overlay = bgr.copy()
            cv2.fillPoly(overlay, [roi_poly], (0,255,0))
            vis_roi = cv2.addWeighted(overlay, 0.25, bgr, 0.75, 0)
            cv2.polylines(vis_roi, [roi_poly], True, (0,0,0), 2)

            bev = self.bev_warp(bgr, roi_poly)

            # 자동 스케일(옵션)
            if (self.xm_per_pix <= 0 or self.ym_per_pix <= 0) and not self._scale_set:
                self.ym_per_pix = float(self.reference_len_m) / float(h)
                self.xm_per_pix = self.ym_per_pix
                self._scale_set = True
                rospy.loginfo(f"[lane_lite] autoscale xm,ym = {self.xm_per_pix:.6f} m/px (H={h}px, ref={self.reference_len_m}m)")

            # 이진화 → 슬윈/메트릭
            bev_bin = self.to_binary(bev)
            (viz, lmean, rmean, kL, kR, kC, kLm, kRm, kCm) = self.sliding_window_and_metrics(bev_bin)


            # (선택) 간단 퍼블리시
            from std_msgs.msg import Float32MultiArray
            out = Float32MultiArray()
            out.data = [lmean, rmean, kL, kR, kC]
            self.pub_vals.publish(out)

            # 시각화 창
            if self.show_window:
                # 좌측: ROI, 우측: BEV/검출
                canvas = np.hstack([cv2.resize(vis_roi, (w, h)), cv2.resize(viz, (w, h))])
                cv2.imshow(self.win_main, canvas)
                cv2.waitKey(1)

        except Exception as e:
            rospy.logwarn(f"[lane_lite] exception: {e}")

    def spin(self):
        rospy.loginfo("lane_lite_node running...")
        rospy.spin()

if __name__ == "__main__":
    node = LaneLiteNode()
    node.spin()
