#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
traffic_sign.py
  - YOLO로 바운딩 박스를 얻은 뒤, 박스 상단 절반의 좌/우 흰색 픽셀 수를 비교해
    좌/우 표지판을 판단하고 Bool 토픽으로 퍼블리시한다.
  - ORB/SIFT 매칭 없이 단순 픽셀 카운트 + 디바운스.
"""
import os
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

# Blue color filter (HSV)
BLUE_LOWER = (100, 80, 50)
BLUE_UPPER = (140, 255, 255)
BLUE_RATIO_MIN = 0.02  # 2% 이상 파란색이 있을 때만 처리

class TrafficSignNode:
    def __init__(self):
        # Parameters
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_rect_color")
        self.left_topic = rospy.get_param("~left_topic", "/ab_sign/left")
        self.right_topic = rospy.get_param("~right_topic", "/ab_sign/right")
        self.debug_topic = rospy.get_param("~debug_topic", "/ab_sign/image_debug")
        self.enable_topic = rospy.get_param("~enable_topic", "/ab_sign/enable")

        self.debounce_frames = int(rospy.get_param("~debounce_frames", 5))
        self.publish_debug = bool(rospy.get_param("~publish_debug", False))
        self.show_window = bool(rospy.get_param("~show_window", True))
        self.enable_duration = float(rospy.get_param("~enable_duration_sec", 8.0)) # 이 traffic_sign을 인지하도록 이 노드가 켜지는 시간에 해당

        # YOLO settings
        self.yolo_model_path = rospy.get_param(
            "~yolo_model_path",
            "/root/autorace_kkk_ws/src/perception_node/src/ab_sign/best_crop.pt",
        )
        self.yolo_conf = float(rospy.get_param("~yolo_conf", 0.25))
        self.yolo_crop_margin = int(rospy.get_param("~yolo_crop_margin", 12))

        # ROS I/O
        self.bridge = CvBridge()
        self.pub_left = rospy.Publisher(self.left_topic, Bool, queue_size=1)
        self.pub_right = rospy.Publisher(self.right_topic, Bool, queue_size=1)
        self.debug_pub = None
        if self.publish_debug:
            self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)

        rospy.Subscriber(self.image_topic, Image, self.cb_image, queue_size=1, buff_size=2**24)
        rospy.Subscriber(self.enable_topic, Bool, self.cb_enable, queue_size=1)
        rospy.loginfo("[traffic_sign] subscribe image -> %s", self.image_topic)
        rospy.loginfo("[traffic_sign] subscribe enable -> %s", self.enable_topic)
        rospy.loginfo("[traffic_sign] publish left -> %s", self.left_topic)
        rospy.loginfo("[traffic_sign] publish right -> %s", self.right_topic)

        # Debounce streaks
        self.streak_left = 0
        self.streak_right = 0
        self.enabled = False
        self.enable_until = None

        # Load YOLO
        self.yolo_model = None
        if YOLO is None:
            rospy.logwarn("[traffic_sign] ultralytics not installed; detection disabled")
        elif not os.path.exists(self.yolo_model_path):
            rospy.logwarn("[traffic_sign] YOLO model not found at %s", self.yolo_model_path)
        else:
            try:
                self.yolo_model = YOLO(self.yolo_model_path)
                rospy.loginfo("[traffic_sign] YOLO loaded: %s", self.yolo_model_path)
            except Exception as e:
                rospy.logwarn("[traffic_sign] failed to load YOLO model: %s", e)

        if self.show_window:
            cv2.namedWindow("ab_sign_debug", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("ab_sign_debug", 960, 540)

    def _debounce(self, is_left, detected):
        if is_left:
            self.streak_left = min(self.streak_left + 1, self.debounce_frames) if detected else max(self.streak_left - 1, 0)
            return self.streak_left >= self.debounce_frames
        else:
            self.streak_right = min(self.streak_right + 1, self.debounce_frames) if detected else max(self.streak_right - 1, 0)
            return self.streak_right >= self.debounce_frames

    def cb_enable(self, msg):
        self.enabled = bool(msg.data)
        if self.enabled and self.enable_duration > 0:
            self.enable_until = rospy.Time.now() + rospy.Duration.from_sec(self.enable_duration)
        else:
            self.enable_until = None
        if not self.enabled:
            # reset states and publish false to clear
            self.streak_left = 0
            self.streak_right = 0
            self.pub_left.publish(Bool(data=False))
            self.pub_right.publish(Bool(data=False))
        rospy.loginfo("[traffic_sign] enabled=%s until=%s",
                      self.enabled,
                      str(self.enable_until.to_sec()) if self.enable_until is not None else "None")

    def cb_image(self, msg):
        # gate by enable flag and optional timeout
        if not self.enabled:
            return
        if self.enable_until is not None and rospy.Time.now() > self.enable_until:
            self.enabled = False
            self.streak_left = 0
            self.streak_right = 0
            self.pub_left.publish(Bool(data=False))
            self.pub_right.publish(Bool(data=False))
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if frame is None:
            return
        debug_frame = frame.copy()

        detected_left = False
        detected_right = False

        if self.yolo_model is None:
            rospy.logwarn_throttle(2.0, "[traffic_sign] YOLO model unavailable")
        else:
            try:
                results = self.yolo_model(frame, verbose=False, conf=self.yolo_conf)
                h, w = frame.shape[:2]
                for res in results:
                    if not hasattr(res, "boxes") or res.boxes is None:
                        continue
                    names = res.names if hasattr(res, "names") else {}
                    for i, box in enumerate(res.boxes.xyxy.cpu().numpy()):
                        cls_id = int(res.boxes.cls[i].item()) if res.boxes.cls is not None else -1
                        cls_name = names.get(cls_id, str(cls_id))
                        x1, y1, x2, y2 = box
                        x1 = int(max(0, x1 - self.yolo_crop_margin))
                        y1 = int(max(0, y1 - self.yolo_crop_margin))
                        x2 = int(min(w - 1, x2 + self.yolo_crop_margin))
                        y2 = int(min(h - 1, y2 + self.yolo_crop_margin))
                        if x2 <= x1 or y2 <= y1:
                            continue
                        roi_bgr = frame[y1:y2, x1:x2]
                        if roi_bgr.size == 0:
                            continue
                        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
                        mask_blue = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
                        blue_ratio = float(cv2.countNonZero(mask_blue)) / float(max(mask_blue.size, 1))
                        if blue_ratio < BLUE_RATIO_MIN:
                            continue
                        roi_gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
                        _, bin_roi = cv2.threshold(roi_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                        h_roi, w_roi = bin_roi.shape[:2]
                        top = bin_roi[:max(1, h_roi // 2), :]
                        mid = w_roi // 2
                        left_count = cv2.countNonZero(top[:, :mid])
                        right_count = cv2.countNonZero(top[:, mid:])
                        decision = "LEFT" if left_count > right_count else "RIGHT"
                        if decision == "LEFT":
                            detected_left = True
                        else:
                            detected_right = True

                        if self.publish_debug or self.show_window:
                            cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                            label = f"{cls_name} {decision} L:{left_count} R:{right_count} B:{blue_ratio:.2f}"
                            cv2.putText(debug_frame, label, (x1, max(y1 - 5, 15)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            except Exception as e:
                rospy.logwarn_throttle(2.0, "[traffic_sign] YOLO inference failed: %s", e)

        # Debounce and publish
        left_state = self._debounce(True, detected_left)
        right_state = self._debounce(False, detected_right)
        self.pub_left.publish(Bool(data=left_state))
        self.pub_right.publish(Bool(data=right_state))

        # Debug publish
        if self.publish_debug and self.debug_pub is not None:
            dbg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            dbg.header = msg.header
            self.debug_pub.publish(dbg)

        if self.show_window:
            cv2.imshow("ab_sign_debug", debug_frame)
            cv2.waitKey(1)


def main():
    rospy.init_node("traffic_sign_node")
    node = TrafficSignNode()
    rospy.spin()


if __name__ == "__main__":
    main()
