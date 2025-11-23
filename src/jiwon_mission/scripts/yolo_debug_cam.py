#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import threading


class YoloImageNode:
    def __init__(self):
        # YOLO 모델
        weight_path = "/root/autorace_kkk_ws/src/yolo/best.pt"
        rospy.loginfo(f"[YOLO] 모델 로딩: {weight_path}")
        self.model = YOLO(weight_path)

        self.bridge = CvBridge()
        self.image_topic = "/usb_cam/image_rect_color"

        # 최신 프레임 저장용
        self.latest_frame = None
        self.lock = threading.Lock()

        # 윈도우 생성
        cv2.namedWindow("YOLO result", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("YOLO result", 800, 600)

        # subscriber
        self.sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        rospy.loginfo(f"[YOLO] Subscribe: {self.image_topic}")

    def image_callback(self, msg):
        """카메라 콜백: 최신 프레임만 저장"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logwarn(f"CvBridge 에러: {e}")
            return

        with self.lock:
            self.latest_frame = frame

    def spin(self):
        """메인 루프: 최신 프레임으로만 YOLO + imshow"""
        rate = rospy.Rate(20)  # GUI용 루프 주기 (원하면 줄여도 됨)

        while not rospy.is_shutdown():
            frame = None
            with self.lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()

            if frame is not None:
                # YOLO 추론
                results = self.model(frame, conf=0.5)[0]

                # bbox 그리기
                for box in results.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    cls_id = int(box.cls)
                    conf = float(box.conf)
                    cls_name = self.model.names[cls_id] if self.model.names else str(cls_id)

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"{cls_name} {conf:.2f}"
                    cv2.putText(
                        frame, label,
                        (x1, max(y1 - 5, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2
                    )

                # 보기 좋게 축소
                h, w = frame.shape[:2]
                scale = 0.5
                disp = cv2.resize(frame, (int(w * scale), int(h * scale)))

                cv2.imshow("YOLO result", disp)
                key = cv2.waitKey(1) & 0xFF

                # 창 닫기 또는 q 누르면 종료
                if key == ord('q'):
                    rospy.loginfo("[YOLO] q 키 입력 → 노드 종료")
                    break

                # 사용자가 창을 X 눌러서 닫았는지 체크 (선택사항)
                if cv2.getWindowProperty("YOLO result", cv2.WND_PROP_VISIBLE) < 1:
                    rospy.loginfo("[YOLO] 창이 닫혀서 종료")
                    break

            rate.sleep()

        cv2.destroyAllWindows()


def main():
    rospy.init_node("yolo_debug_cam", anonymous=False)
    node = YoloImageNode()
    rospy.loginfo("[YOLO] 이미지 디버그 노드 시작")
    node.spin()


if __name__ == "__main__":
    main()
