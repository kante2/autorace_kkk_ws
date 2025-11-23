#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import cv2
import threading


class YoloImageRectViewer:
    def __init__(self):

        weight_path = "/root/autorace_kkk_ws/src/yolo/best.pt"
        rospy.loginfo(f"[YOLO] loading model: {weight_path}")
        self.model = YOLO(weight_path)

        # ✅ 모델 클래스 이름 확인용 로그
        rospy.loginfo(f"[YOLO] model.names = {self.model.names}")

        self.bridge = CvBridge()

        # 구독할 이미지 토픽
        self.image_topic = "/usb_cam/image_rect_color"
        rospy.loginfo(f"[YOLO] subscribe: {self.image_topic}")
        self.sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        # 최신 프레임 저장용 (콜백 ↔ 메인루프 공유)
        self.latest_frame = None
        self.lock = threading.Lock()

        # OpenCV 창 설정
        cv2.namedWindow("YOLO Steering (debug)", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("YOLO Steering (debug)", 800, 600)

    # ---------------- 콜백: 이미지 수신 ----------------
    def image_callback(self, msg):
        rospy.loginfo_once("[YOLO] first image arrived")

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error: {e}")
            return

        # 필요하다 싶으면 여기서 flip 테스트 가능
        # frame = cv2.flip(frame, 1)

        with self.lock:
            self.latest_frame = frame

    # ---------------- 메인 루프: YOLO + imshow ---------
    def spin(self):
        rate = rospy.Rate(20)  # GUI 루프 Hz

        while not rospy.is_shutdown():
            frame = None
            with self.lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()

            if frame is not None:
                try:
                    # YOLO 추론
                    results = self.model(frame, conf=0.5)
                    boxes = results[0].boxes

                    # ✅ 디버깅: 현재 프레임에서 검출된 클래스/이름/신뢰도 출력
                    if boxes is not None and len(boxes) > 0:
                        cls_list = []
                        for b in boxes:
                            cls = int(b.cls[0])
                            conf = float(b.conf[0])
                            name = self.model.names.get(cls, "UNKNOWN")
                            cls_list.append(cls)
                            rospy.loginfo_throttle(
                                0.5,
                                f"[YOLO] det cls={cls}, name={name}, conf={conf:.2f}"
                            )
                        rospy.loginfo_throttle(
                            0.5,
                            f"[YOLO] cls list in frame: {cls_list}"
                        )
                    else:
                        rospy.loginfo_throttle(1.0, "[YOLO] no boxes detected")

                    annotated = results[0].plot()

                except Exception as e:
                    rospy.logerr(f"[YOLO] inference error: {e}")
                    annotated = frame

                cv2.imshow("YOLO Steering (debug)", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    rospy.loginfo("ESC pressed, shutting down node.")
                    rospy.signal_shutdown("user quit")
                    break

            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("yolo_image_rect_viewer")
    node = YoloImageRectViewer()
    node.spin()
