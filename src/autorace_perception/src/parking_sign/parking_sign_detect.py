#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

class ParkingSignORBNode(object):
    def __init__(self):
        # 파라미터
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_rect_color')
        self.img_path = rospy.get_param(
            '~img_path',
            '/root/autorace_kkk_ws/src/autorace_perception/src/parking_sign/Parking_sign.jpg'
        )
        self.resize_template_max = int(rospy.get_param('~resize_template_max', 512))
        self.orb_nfeatures = int(rospy.get_param('~orb_nfeatures', 1000))
        self.lowe_ratio = float(rospy.get_param('~lowe_ratio', 0.75))
        self.ransac_reproj_thresh = float(rospy.get_param('~ransac_reproj_thresh', 3.0))
        self.min_matches = int(rospy.get_param('~min_matches', 20))
        self.min_inlier_ratio = float(rospy.get_param('~min_inlier_ratio', 0.25))
        self.debounce_frames = int(rospy.get_param('~debounce_frames', 5))
        self.publish_debug = bool(rospy.get_param('~publish_debug', False))
        self.show_window = bool(rospy.get_param('~show_window', True))

        # 템플릿 로드 및 전처리
        if not os.path.exists(self.img_path):
            raise IOError('img_path not found: {}'.format(self.img_path))
        tmpl_gray = cv2.imread(self.img_path, cv2.IMREAD_GRAYSCALE)
        if tmpl_gray is None:
            raise IOError('failed to read template image')
        th, tw = tmpl_gray.shape[:2]
        scale = min(1.0, float(self.resize_template_max) / max(th, tw))
        if scale < 1.0:
            tmpl_gray = cv2.resize(tmpl_gray, (int(tw*scale), int(th*scale)), interpolation=cv2.INTER_AREA)
        self.template_gray = tmpl_gray
        self.template_h, self.template_w = self.template_gray.shape[:2]

        # ORB와 매처
        self.orb = cv2.ORB_create(nfeatures=self.orb_nfeatures, scaleFactor=1.2, nlevels=8)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # 템플릿 특징 미리 계산
        self.kp_t, self.des_t = self.orb.detectAndCompute(self.template_gray, None)
        if self.des_t is None or len(self.kp_t) < 10:
            raise RuntimeError('not enough keypoints in template')

        # ROS I/O
        self.bridge = CvBridge()
        self.flag_pub = rospy.Publisher('/parking_sign/flag', Bool, queue_size=10)
        self.debug_pub = None
        if self.publish_debug:
            self.debug_pub = rospy.Publisher('/parking_sign/image_debug', Image, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.cb_image, queue_size=1, buff_size=2**24)

        self.hit_streak = 0
        rospy.loginfo('parking_sign_orb_node started')

    def cb_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #cv2.imshow("img",gray)

        kp_f, des_f = self.orb.detectAndCompute(gray, None)
        detected = False
        if des_f is not None and len(kp_f) >= 10:
            knn = self.matcher.knnMatch(self.des_t, des_f, k=2)
            good = []
            for m, n in knn:
                if m.distance < self.lowe_ratio * n.distance:
                    good.append(m)

            if len(good) >= self.min_matches:
                src = np.float32([self.kp_t[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                dst = np.float32([kp_f[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
                H, mask = cv2.findHomography(src, dst, cv2.RANSAC, self.ransac_reproj_thresh)
                if H is not None and mask is not None:
                    inliers = int(mask.sum())
                    ratio = inliers / float(max(len(good), 1))
                    if inliers >= self.min_matches and ratio >= self.min_inlier_ratio:
                        detected = True
                        if self.publish_debug:
                            box = np.float32([[0, 0], [self.template_w, 0],
                                              [self.template_w, self.template_h], [0, self.template_h]]).reshape(-1, 1, 2)
                            poly = cv2.perspectiveTransform(box, H)
                            cv2.polylines(frame, [np.int32(poly)], True, (0, 255, 0), 3)
                            cv2.putText(frame, 'inliers {} ratio {:.2f}'.format(inliers, ratio),
                                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 디바운싱
        if detected:
            self.hit_streak = min(self.hit_streak + 1, self.debounce_frames)
            print("detected")
            
        else:
            self.hit_streak = max(self.hit_streak - 1, 0)
            print("not detected")
        flag = Bool(data=(self.hit_streak >= self.debounce_frames))
        self.flag_pub.publish(flag)

        if self.publish_debug and self.debug_pub is not None:
            dbg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            dbg.header = msg.header
            self.debug_pub.publish(dbg)

        if self.show_window:
            cv2.imshow('parking_sign_debug', frame)
            cv2.waitKey(1)

def main():
    rospy.init_node('parking_sign_orb_node')
    node = ParkingSignORBNode()
    rospy.spin()

if __name__ == '__main__':
    main()
