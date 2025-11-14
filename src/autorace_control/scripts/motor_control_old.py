#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage 
# 
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped



class LaneCurvatureNode:
    def __init__(self):
        rospy.init_node("lane_curvature_node")
        rospy.loginfo("lane_curvature_node started")

        # === UI ===
        self.show_window = rospy.get_param("~show_window", True)
        self.win_src = "src_with_roi"
        self.win_bev = "bev_binary_and_windows"
        if self.show_window:
            try: cv2.startWindowThread()
            except Exception: pass
            cv2.namedWindow(self.win_src, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.win_src, 960, 540)
            cv2.namedWindow(self.win_bev, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.win_bev, 960, 540)

        # === ROS IO ===
        self.bridge = CvBridge()
        # self.sub = rospy.Subscriber("/usb_cam/image_rect_color", Image, self.cb_image,
        #                             queue_size=2, buff_size=2**24)
        self.sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cb_image,
                                    queue_size=2, buff_size=2**24)

        # pub --> controller
        # 1.곡률
        self.pub_k_left   = rospy.Publisher("/perception/curvature_left",   Float32, queue_size=1)
        self.pub_k_right  = rospy.Publisher("/perception/curvature_right",  Float32, queue_size=1)
        self.pub_k_center = rospy.Publisher("/perception/curvature_center", Float32, queue_size=1)
        # 2.차선 중심
        # self.pub_center_point = rospy.Publisher("/perception/center_point_px", PointStamped, queue_size=1)
        self.pub_center_x = rospy.Publisher("/perception/center_x_px", Float32, queue_size=1)
        self.pub_dx = rospy.Publisher("/perception/dx_px", Float32, queue_size=1)

        self.lane_width_px = rospy.get_param("~lane_width_px", 480.0)  # BEV에서의 차선 폭(px) 추정치


        # === Sliding-window params ===
        self.num_windows = rospy.get_param("~num_windows", 12)
        self.window_margin = rospy.get_param("~window_margin", 80)
        self.minpix_recenter = rospy.get_param("~minpix_recenter", 50)
        self.min_lane_sep = rospy.get_param("~min_lane_sep", 60)   # 좌/우 창 간 최소 분리(px)
        self.center_ema_alpha = rospy.get_param("~center_ema_alpha", 0.8)

        # === ROI polygon (ratios) ===
        # 사다리꼴 ROI (OpenCV 좌표: y down)
        self.roi_top_y_ratio     = rospy.get_param("~roi_top_y_ratio", 0.60)
        self.roi_left_top_ratio  = rospy.get_param("~roi_left_top_ratio", 0.22)
        self.roi_right_top_ratio = rospy.get_param("~roi_right_top_ratio", 0.78)
        self.roi_left_bot_ratio  = rospy.get_param("~roi_left_bot_ratio", -0.40)  # 화면 밖까지 확장 가능
        self.roi_right_bot_ratio = rospy.get_param("~roi_right_bot_ratio", 1.40)

        # === Color thresholds (HSV) ===
        # self.yellow_lower = np.array([10,  80,  60], dtype=np.uint8)
        # self.yellow_upper = np.array([45, 255, 255], dtype=np.uint8)
        # self.white_lower  = np.array([ 0,   0, 150], dtype=np.uint8)
        # self.white_upper  = np.array([179,  60, 255], dtype=np.uint8)
        
        # morai
        self.yellow_lower = np.array([18,  60,  140], dtype=np.uint8)
        self.yellow_upper = np.array([40, 255, 255], dtype=np.uint8)
        self.white_lower  = np.array([ 0,   0, 150], dtype=np.uint8)
        self.white_upper  = np.array([179,  60, 255], dtype=np.uint8)

    # ---------------- Core helpers ----------------
    def make_roi_polygon(self, h, w):
        """사다리꼴 ROI 폴리곤 생성 (BL, TL, TR, BR; OpenCV y-down)"""
        y_top = int(h * self.roi_top_y_ratio)
        y_bot = h - 1
        x_lt  = int(w * self.roi_left_top_ratio)
        x_rt  = int(w * self.roi_right_top_ratio)
        x_lb  = int(w * self.roi_left_bot_ratio)
        x_rb  = int(w * self.roi_right_bot_ratio)
        # 하단 x는 화면 바깥을 허용하지만 워프 전에 y만 클리핑
        return np.array([[x_lb, y_bot], [x_lt, y_top], [x_rt, y_top], [x_rb, y_bot]], np.int32)

    def warp_to_bev(self, bgr, roi_poly):
        """ROI 사다리꼴을 이미지 전체 직사각형으로 펴서(BEV) 반환"""
        h, w = bgr.shape[:2]
        BL, TL, TR, BR = roi_poly.astype(np.float32)

        # y는 유효 영역으로 클리핑 (x는 일부 화면 밖을 허용)
        for p in (BL, TL, TR, BR):
            p[1] = np.clip(p[1], 0, h - 1)

        src = np.float32([BL, TL, TR, BR])
        dst = np.float32([[0, h-1], [0, 0], [w-1, 0], [w-1, h-1]])  # 전체 프레임로 펴기
        M = cv2.getPerspectiveTransform(src, dst)
        bev = cv2.warpPerspective(bgr, M, (w, h),
                                  flags=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT,
                                  borderValue=(0, 0, 0))
        return bev

    def binarize_lanes(self, bgr):
        """노랑/흰색 차선 마스크 합성 (HSV 기반)"""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask_y = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        mask_w = cv2.inRange(hsv, self.white_lower,  self.white_upper)
        kernel = np.ones((3, 3), np.uint8)
        mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_OPEN, kernel, iterations=1)
        mask_w = cv2.morphologyEx(mask_w, cv2.MORPH_OPEN, kernel, iterations=1)
        return cv2.bitwise_or(mask_y, mask_w)

    def run_sliding_window_collect_centers(self, binary_mask):
        """
        슬라이딩 윈도우로 좌/우 차선 픽셀을 모으고,
        각 층(window band)에서 중심점(x_mean)을 구해 좌/우 리스트에 저장.
        (OpenCV 좌표계: (x 오른쪽+, y 아래+))
        """
        h, w = binary_mask.shape[:2]
        nonzero = binary_mask.nonzero()
        nz_y = np.array(nonzero[0]); nz_x = np.array(nonzero[1])

        # 하단 절반 히스토그램으로 초기 좌/우 베이스 x
        histogram = np.sum(binary_mask[h//2:, :], axis=0)
        midpoint = w // 2
        left_base = np.argmax(histogram[:midpoint]) if histogram[:midpoint].any() else None
        right_base = (np.argmax(histogram[midpoint:]) + midpoint) if histogram[midpoint:].any() else None

        # 디버그 캔버스
        debug_img = cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2BGR)
        window_height = int(h / self.num_windows)

        left_current = left_base
        right_current = right_base
        left_indices = []
        right_indices = []

        left_window_centers = []   # [(y_center, x_center), ...]
        right_window_centers = []

        for win in range(self.num_windows):
            y_low = h - (win + 1) * window_height
            y_high = h - win * window_height

            # 창 그리기
            if left_current is not None:
                cv2.rectangle(debug_img,
                              (left_current - self.window_margin, y_low),
                              (left_current + self.window_margin, y_high),
                              (255, 0, 0), 2)
            if right_current is not None:
                cv2.rectangle(debug_img,
                              (right_current - self.window_margin, y_low),
                              (right_current + self.window_margin, y_high),
                              (255, 0, 0), 2)

            # 포인트 수집
            good_left = []
            good_right = []
            if left_current is not None:
                good_left = ((nz_y >= y_low) & (nz_y < y_high) &
                             (nz_x >= left_current - self.window_margin) &
                             (nz_x <  left_current + self.window_margin)).nonzero()[0].tolist()
            if right_current is not None:
                good_right = ((nz_y >= y_low) & (nz_y < y_high) &
                              (nz_x >= right_current - self.window_margin) &
                              (nz_x <  right_current + self.window_margin)).nonzero()[0].tolist()

            # 좌/우가 너무 붙었을 때 한쪽 억제
            if left_current is not None and right_current is not None:
                if abs(left_current - right_current) < self.min_lane_sep:
                    if len(good_left) < len(good_right): good_left = []
                    else: good_right = []

            left_indices.extend(good_left)
            right_indices.extend(good_right)

            # 층 중앙 y
            y_center = (y_low + y_high) // 2

            # 중심점(평균 x) 계산 & 시각화
            if len(good_left) > 0:
                x_mean_left = float(np.mean(nz_x[good_left]))
                left_window_centers.append((int(y_center), float(x_mean_left)))
                cv2.circle(debug_img, (int(x_mean_left), int(y_center)), 4, (0, 0, 255), -1)  # 빨강
            if len(good_right) > 0:
                x_mean_right = float(np.mean(nz_x[good_right]))
                right_window_centers.append((int(y_center), float(x_mean_right)))
                cv2.circle(debug_img, (int(x_mean_right), int(y_center)), 4, (0, 255, 255), -1)  # 노랑

            # 창 중심 EMA 업데이트
            if len(good_left) > self.minpix_recenter and left_current is not None:
                left_current = int(self.center_ema_alpha * left_current +
                                   (1 - self.center_ema_alpha) * float(np.mean(nz_x[good_left])))
            if len(good_right) > self.minpix_recenter and right_current is not None:
                right_current = int(self.center_ema_alpha * right_current +
                                    (1 - self.center_ema_alpha) * float(np.mean(nz_x[good_right])))

        # 수집된 픽셀을 색칠(디버그)
        if len(left_indices) > 0:
            lx = np.clip(nz_x[left_indices], 0, w-1)
            ly = np.clip(nz_y[left_indices], 0, h-1)
            debug_img[ly, lx] = (0, 0, 255)
        if len(right_indices) > 0:
            rx = np.clip(nz_x[right_indices], 0, w-1)
            ry = np.clip(nz_y[right_indices], 0, h-1)
            debug_img[ry, rx] = (0, 255, 0)

        return debug_img, left_window_centers, right_window_centers

    def compute_curvature_from_centers(self, centers, image_height):
        """
        중심점들( (y, x) 리스트 )만을 사용해서 2차 다항 x(y)=ay^2+by+c 피팅 후
        하단 y=h-1에서 곡률 kappa = |x''| / (1 + x'^2)^(3/2) 계산. (단위: 1/px)
        중심점이 너무 적으면 None 반환.
        """
        if len(centers) < 5:
            return None, None  # (fit, curvature)
        ys = np.array([p[0] for p in centers], dtype=np.float64)
        xs = np.array([p[1] for p in centers], dtype=np.float64)

        fit = np.polyfit(ys, xs, 2)  # x = a y^2 + b y + c
        a, b, c = fit
        y_eval = float(image_height - 1)
        dxdy = 2*a*y_eval + b
        d2xdy2 = 2*a
        curvature = abs(d2xdy2) / ((1.0 + dxdy*dxdy) ** 1.5)  # [1/px]
        return fit, curvature

    def compute_center_point(self, left_window_centers, right_window_centers, image_height):
        """
        대표점(좌/우)을 '전체 윈도우 평균'으로 계산한 뒤,
        - 양쪽 있으면: 두 대표점의 단순 평균 → 중앙
        - 한쪽만 있으면: lane_width_px/2 만큼 보정해 중앙 추정
        반환: (center_y, center_x)  [OpenCV/BEV 픽셀 좌표]
        """
        def side_mean(centers):
            if not centers:
                return None
            arr = np.array(centers, dtype=np.float64)  # shape (N,2): [y, x]
            y_mean = float(np.mean(arr[:, 0]))
            x_mean = float(np.mean(arr[:, 1]))
            return (int(round(y_mean)), x_mean)

        # 좌/우 대표점(평균)
        left_rep  = side_mean(left_window_centers)
        right_rep = side_mean(right_window_centers)

        # 차선 폭 절반(px) — 파라미터로 받아두는 걸 권장 (예: 340px 기준)
        half_w = 0.5 * float(self.lane_width_px)

        # 둘 다 보이면: 단순 평균
        if left_rep is not None and right_rep is not None:
            cy = int(round(0.5 * (left_rep[0] + right_rep[0])))
            cx = 0.5 * (left_rep[1] + right_rep[1])
            return (cy, cx)

        # 한쪽만 보이는 경우: 보정해서 중앙 추정
        if left_rep is not None:
            return (left_rep[0], left_rep[1] + half_w)   # 좌선에서 오른쪽으로 half_w
        if right_rep is not None:
            return (right_rep[0], right_rep[1] - half_w) # 우선에서 왼쪽으로 half_w

        # 아무것도 없으면
        return None

            

    def draw_polynomial(self, canvas, fit, color=(255, 255, 0), step=10):
        """피팅된 x(y)를 캔버스 위에 점으로 표시"""
        h = canvas.shape[0]
        ploty = np.arange(0, h, step, dtype=np.int32)
        fitx = (fit[0]*ploty**2 + fit[1]*ploty + fit[2]).astype(np.int32)
        for y, x in zip(ploty, fitx):
            cv2.circle(canvas, (int(x), int(y)), 2, color, -1)

    # ---------------- ROS callback ----------------
    def cb_image(self, msg):
        try:
            # bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # morai version,, 
            bgr = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if bgr is None: return
            if bgr.ndim == 2 or (bgr.ndim == 3 and bgr.shape[2] == 1):
                bgr = cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR)

            h, w = bgr.shape[:2]

            # 1) ROI 폴리곤 & 시각화
            roi_poly = self.make_roi_polygon(h, w)
            src_vis = bgr.copy()
            overlay = bgr.copy()
            cv2.fillPoly(overlay, [roi_poly], (0, 255, 0))
            src_vis = cv2.addWeighted(overlay, 0.25, bgr, 0.75, 0)
            cv2.polylines(src_vis, [roi_poly], True, (0, 0, 0), 2)

            # 2) BEV (ROI 전체를 프레임 전체로 펴기)
            bev_bgr = self.warp_to_bev(bgr, roi_poly)

            # 3) 이진화
            bev_binary = self.binarize_lanes(bev_bgr)

            # 4) 슬라이딩 윈도우로 중심점 수집
            debug_img, left_window_centers, right_window_centers = \
                self.run_sliding_window_collect_centers(bev_binary)

            # 5) 중심점만으로 곡률 계산 (좌/우)
            # left_fit: 좌측 차선 2차 다항 계수 : x_left(y)  = a_L*y*y + b_L*y + c_L
            # right_fit: 우측 차선 2차 다항 계수 : x_right(y) = a_R*y*y + b_R*y + c_R
            left_fit, left_curvature = self.compute_curvature_from_centers(
                left_window_centers, image_height=bev_binary.shape[0])
            right_fit, right_curvature = self.compute_curvature_from_centers(
                right_window_centers, image_height=bev_binary.shape[0])
        

            # 6) (선택) 중앙선 = 좌/우 평균 (둘 다 있을 때)
            center_fit = None
            center_curvature = None
            if left_fit is not None and right_fit is not None:
                center_fit = 0.5*(left_fit + right_fit)
                # 중앙선 곡률
                a, b, c = center_fit
                y_eval = float(bev_binary.shape[0] - 1)
                dxdy = 2*a*y_eval + b
                d2xdy2 = 2*a
                center_curvature = abs(d2xdy2) / ((1.0 + dxdy*dxdy) ** 1.5)

            def curv_msg(v): 
                return Float32(data=float(v)) if (v is not None and np.isfinite(float(v))) else Float32(data=float('nan'))

            self.pub_k_left.publish(curv_msg(left_curvature))
            self.pub_k_right.publish(curv_msg(right_curvature))
            self.pub_k_center.publish(curv_msg(center_curvature))

            center_point = self.compute_center_point(left_window_centers, right_window_centers, bev_binary.shape[0])
            if center_point is not None:
                cy, cx = center_point
                pt_msg = PointStamped()
                pt_msg.header.stamp = msg.header.stamp
                pt_msg.header.frame_id = "bev"   # ★ BEV 픽셀 좌표계 명시 (x→오른쪽, y→아래)
                pt_msg.point.x = float(cx)
                pt_msg.point.y = float(cy)
                pt_msg.point.z = 0.0
                # self.pub_center_point.publish(pt_msg)
                cv2.circle(debug_img, (int(cx), int(cy)), 6, (255, 0, 255), -1)

                # === 디버그: 중앙 픽셀 vs 카메라(프레임) 중심 표시 ===
                img_cx = bev_binary.shape[1] * 0.5
                img_cy = bev_binary.shape[0] * 0.5

                # 규약 고정: dx = cx - img_cx  (오른쪽이 +)
                dx = float(cx) - float(img_cx)
                dy = float(cy) - float(img_cy)

                # ROS 퍼블리시
                self.pub_center_x.publish(Float32(data=float(cx)))
                self.pub_dx.publish(Float32(data=float(dx)))

                # BEV 디버그 이미지에: 카메라(프레임) 중심 십자 + 차선 중심 십자/보라점
                cv2.drawMarker(debug_img, (int(img_cx), int(img_cy)), (255, 255, 0),  # 시안색
                            markerType=cv2.MARKER_CROSS, markerSize=14, thickness=2)
                cv2.drawMarker(debug_img, (int(cx), int(cy)), (255, 0, 255),
                            markerType=cv2.MARKER_TILTED_CROSS, markerSize=12, thickness=2)

                # 가이드 라인
                cv2.line(debug_img, (int(cx), 0), (int(cx), bev_binary.shape[0]-1), (255, 0, 255), 1)
                cv2.line(debug_img, (0, int(cy)), (bev_binary.shape[1]-1, int(cy)), (255, 0, 255), 1)

                # 텍스트(좌표 & 오프셋)
                cv2.putText(debug_img,
                            f"LaneCenter=({cx:.1f},{cy:.1f})  ImgCenter=({img_cx:.1f},{img_cy:.1f})  d=({dx:+.1f},{dy:+.1f})",
                            (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)

                # 좌측 원본(src_vis)에도 카메라 프레임 중심을 동일하게 표시(시안색)
                src_cx = w * 0.5
                src_cy = h * 0.5
                cv2.drawMarker(src_vis, (int(src_cx), int(src_cy)), (255, 255, 0),
                            markerType=cv2.MARKER_CROSS, markerSize=14, thickness=2)

            # --------------------------------------------------------------------------------
                
            # 7) 피팅 시각화 (옵션)
            if left_fit is not None:
                self.draw_polynomial(debug_img, left_fit, (0, 0, 255), step=10)
            if right_fit is not None:
                self.draw_polynomial(debug_img, right_fit, (0, 255, 0), step=10)
            if center_fit is not None:
                self.draw_polynomial(debug_img, center_fit, (255, 0, 255), step=10)

            # 8) 텍스트 오버레이 (cv2 디버그용)
            def put(txt, y):
                cv2.putText(debug_img, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255,255,255), 2, cv2.LINE_AA)

            put(f"Left centers:  {len(left_window_centers)}", 24)
            put(f"Right centers: {len(right_window_centers)}", 48)
            put(f"Curvature Left:{left_curvature if left_curvature is not None else np.nan:.4e} "
                f"Curvature Right:{right_curvature if right_curvature is not None else np.nan:.4e} "
                f"Curvature Center:{center_curvature if center_curvature is not None else np.nan:.4e}", 72)

            # 9) 화면 표시
            if self.show_window:
                canvas = np.hstack([
                    cv2.resize(src_vis, (w, h)),
                    cv2.resize(debug_img, (w, h))
                ])
                cv2.imshow(self.win_src, canvas)       # 좌: ROI오버레이, 우: 디텍션/피팅
                cv2.imshow(self.win_bev, bev_binary)   # BEV 이진화만 단독 확인용
                cv2.waitKey(1)

        except Exception as e:
            rospy.logwarn(f"[lane_curvature_node] exception: {e}")

    def spin(self):
        rospy.loginfo("lane_curvature_node running...")
        rospy.spin()


if __name__ == "__main__":
    LaneCurvatureNode().spin()
