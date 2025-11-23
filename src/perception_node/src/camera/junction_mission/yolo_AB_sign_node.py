#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

import cv2
from ultralytics import YOLO


class YoloTurnSignNode:
    def __init__(self):
        rospy.loginfo("[yolo_turn_sign_node] init")

        # --- 파라미터 ---
        model_path   = rospy.get_param("~model_path", "/root/autorace_kkk_ws/src/yolo/best.pt")
        self.cam_topic = rospy.get_param("~camera_topic", "/usb_cam/image_rect_color")
        self.conf_thres = rospy.get_param("~conf_thres", 0.5)
        self.show_window = rospy.get_param("~show_window", False)

        # 클래스 이름 기준으로 left/right 분류 (모델 이름에 맞게 수정)
        self.left_labels  = rospy.get_param("~left_labels",  ["left", "left_sign"])
        self.right_labels = rospy.get_param("~right_labels", ["right", "right_sign"])

        # YOLO 모델 로드
        rospy.loginfo("[yolo_turn_sign_node] loading model: %s", model_path)
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        # --- Publisher ---
        self.pub_left  = rospy.Publisher("/detect_left_sign",  Bool, queue_size=1)
        self.pub_right = rospy.Publisher("/detect_right_sign", Bool, queue_size=1)

        # --- Subscriber ---
        self.sub_img = rospy.Subscriber(self.cam_topic, Image, self.image_cb, queue_size=1)

        if self.show_window:
            cv2.namedWindow("yolo_sign_debug", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("yolo_sign_debug", 800, 600)

        rospy.loginfo("[yolo_turn_sign_node] ready. Subscribing to %s", self.cam_topic)

    def image_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logwarn("[yolo_turn_sign_node] cv_bridge error: %s", str(e))
            return

        if cv_img is None:
            return

        # --- YOLO 추론 ---
        try:
            results = self.model(cv_img, conf=self.conf_thres)[0]
        except Exception as e:
            rospy.logwarn("[yolo_turn_sign_node] YOLO inference error: %s", str(e))
            return

        left_detected  = False
        right_detected = False

        # 디버그용 이미지
        dbg_img = cv_img.copy()

        # 박스 순회
        for box in results.boxes:
            cls_id = int(box.cls)
            conf   = float(box.conf)

            # 클래스 이름 (model.names는 dict 또는 list)
            cls_name = self.model.names[cls_id] if self.model.names else str(cls_id)

            # left/right 감지
            if cls_name in self.left_labels:
                left_detected = True
            if cls_name in self.right_labels:
                right_detected = True

            # 디버그 박스 그리기
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            color = (0, 255, 0)
            label = f"{cls_name} {conf:.2f}"
            cv2.rectangle(dbg_img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(dbg_img, label, (x1, max(y1 - 5, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # --- 토픽 퍼블리시 ---
        left_msg = Bool()
        right_msg = Bool()
        left_msg.data  = left_detected
        right_msg.data = right_detected

        self.pub_left.publish(left_msg)
        self.pub_right.publish(right_msg)

        # 로그는 너무 자주면 시끄러우니까 THROTTLE
        rospy.loginfo_throttle(1.0,
            "[yolo_turn_sign_node] left=%s right=%s",
            str(left_detected), str(right_detected)
        )

        # 디버그 창
        if self.show_window:
            cv2.imshow("yolo_sign_debug", dbg_img)
            cv2.waitKey(1)


def main():
    rospy.init_node("yolo_turn_sign_node")
    node = YoloTurnSignNode()
    rospy.loginfo("yolo_turn_sign_node running...")
    try:
        rospy.spin()
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()