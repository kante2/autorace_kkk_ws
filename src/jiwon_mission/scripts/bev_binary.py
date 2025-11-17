#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped

# ROI, BEV, BINARY
# PUB: /stopline/bev_binary


class BevBinaryNode:
    def __init__(self):
        rospy.init_node("bev_binary")
        rospy.loginfo("bev_binary info")

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
        self.sub = rospy.Subscriber("/usb_cam/image_rect_color", Image, self.cb_image,
                                    queue_size=2, buff_size=2**24)
        from std_msgs.msg import Float32

        # pub 
        # ROI + Bird Eye View + Binary
        self.pub_bev_binary   = rospy.Publisher("/stopline/bev_binary", Image, queue_size=1)

        # === ROI polygon (ratios) ===
        # 사다리꼴 ROI (OpenCV 좌표: y down)
        self.roi_top_y_ratio     = rospy.get_param("~roi_top_y_ratio", 0.60)
        self.roi_left_top_ratio  = rospy.get_param("~roi_left_top_ratio", 0.22)
        self.roi_right_top_ratio = rospy.get_param("~roi_right_top_ratio", 0.78)
        self.roi_left_bot_ratio  = rospy.get_param("~roi_left_bot_ratio", -0.40)  # 화면 밖까지 확장 가능
        self.roi_right_bot_ratio = rospy.get_param("~roi_right_bot_ratio", 1.40)

        # === Color thresholds (HSV) ===
        #self.yellow_lower = np.array([10,  80,  60], dtype=np.uint8)
        #self.yellow_upper = np.array([45, 255, 255], dtype=np.uint8)
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
        #mask_y = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        mask_w = cv2.inRange(hsv, self.white_lower,  self.white_upper)
        kernel = np.ones((3, 3), np.uint8)
        #mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_OPEN, kernel, iterations=1)
        mask_w = cv2.morphologyEx(mask_w, cv2.MORPH_OPEN, kernel, iterations=1)
        #return cv2.bitwise_or(mask_y, mask_w)
        return mask_w

    # ---------------- ROS callback ----------------
    def cb_image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
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
            bev_binary_u8 = bev_binary.astype(np.uint8)

            # 4) 퍼블리시 (sensor_msgs/Image, mono8)
            bev_msg = self.bridge.cv2_to_imgmsg(bev_binary_u8, encoding="mono8")
            bev_msg.header = msg.header  # 타임스탬프/프레임 동일하게
            self.pub_bev_binary.publish(bev_msg)

            # 9) 화면 표시
            if self.show_window:
                cv2.imshow(self.win_bev, bev_binary_u8)
                cv2.waitKey(1)
        except Exception as e:
            rospy.logwarn(f"[bev_binary] exception: {e}")

    def spin(self):
        rospy.loginfo("bev_binary running...")
        rospy.spin()


if __name__ == "__main__":
    BevBinaryNode().spin()
