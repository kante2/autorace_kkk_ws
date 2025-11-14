#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped


#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np

from ultralytics import YOLO

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray


class Yolov8DetectorNode:
    def __init__(self):
        rospy.init_node("yolov8_detector_node")
        rospy.loginfo("yolov8_detector_node started")

        # === Params ===
        self.model_path = rospy.get_param(
            "~model_path",
            "/root/autorace_kkk_ws/src/yolo/best.pt"
        )
        self.image_topic = rospy.get_param(
            "~image_topic",
            "/usb_cam/image_raw/compressed"   # 필요하면 바꿔
        )
        self.show_window = rospy.get_param("~show_window", True)
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.conf_thres = float(rospy.get_param("~conf_thres", 0.5))

        # 클래스 이름 (원하는 이름으로 바꿔)
        # 3개 클래스라 했으니까 예시로 넣어놓음
        self.class_names = rospy.get_param(
            "~class_names",
            ["turn_left", "turn_right", "parking"]
        )

        # === YOLO 모델 로드 ===
        rospy.loginfo(f"[yolov8] loading model: {self.model_path}")
        self.model = YOLO(self.model_path)

        # === Publisher ===
        # data = [cls, cx, cy, cls, cx, cy, ...]
        self.pub_centers = rospy.Publisher(
            "~detections",
            Float32MultiArray,
            queue_size=1
        )

        # === Subscriber ===
        self.sub_img = rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        if self.show_window:
            cv2.namedWindow("yolo_view", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("yolo_view", 960, 540)

    def image_callback(self, msg: CompressedImage):
        # --- CompressedImage -> BGR 이미지 ---
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn("[yolov8] failed to decode image")
            return

        h, w = frame.shape[:2]

        # --- YOLOv8 추론 ---
        # results[0] : 한 프레임에 대한 결과
        results = self.model(
            frame,
            imgsz=self.imgsz,
            conf=self.conf_thres,
            verbose=False
        )[0]

        centers_flat = []  # [cls, cx, cy, cls, cx, cy, ...]

        if results.boxes is not None and len(results.boxes) > 0:
            for box in results.boxes:
                # xyxy: [x1, y1, x2, y2]
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())

                # 중심 좌표 (카메라 프레임에서의 픽셀)
                cx = 0.5 * (x1 + x2)
                cy = 0.5 * (y1 + y2)

                centers_flat.extend([float(cls_id), float(cx), float(cy)])

                # --- 시각화용 bbox 그리기 ---
                color = (0, 255, 0)
                cv2.rectangle(
                    frame,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    color,
                    2
                )
                cv2.circle(
                    frame,
                    (int(cx), int(cy)),
                    3,
                    (0, 0, 255),
                    -1
                )

                # 클래스 이름 + confidence
                if 0 <= cls_id < len(self.class_names):
                    cls_name = self.class_names[cls_id]
                else:
                    cls_name = f"id{cls_id}"

                label = f"{cls_name} {conf:.2f}"
                cv2.putText(
                    frame,
                    label,
                    (int(x1), int(max(0, y1 - 5))),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2
                )

        # --- 퍼블리시 (클래스 + 중심좌표) ---
        msg_out = Float32MultiArray()
        msg_out.data = centers_flat  # [cls, cx, cy, cls, cx, cy, ...]
        self.pub_centers.publish(msg_out)

        # --- 시각화 ---
        if self.show_window:
            cv2.imshow("yolo_view", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                rospy.signal_shutdown("ESC pressed")

    def spin(self):
        rospy.loginfo("yolov8_detector_node spinning...")
        rospy.spin()
        if self.show_window:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        node = Yolov8DetectorNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass



'''
/root/autorace_kkk_ws/src/yolo/best.pt --> yolov8을 이용할거임

클래스는 3개이고, 바운딩 박스 처리를 cv2 로 시각화 할 예정이다.

1. 클래스에 대한 정보를 퍼블리시 할 것,

2. 바운딩 박스의 카메라 프레임상의 중심을 퍼블리시 할 것,



'''



