#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import yaml
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan


class LidarToImageOverlay:
    def __init__(self):
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_rect_color')
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.calib_yaml = rospy.get_param('~calibration_yaml')
        #hello 

        self.point_step = int(rospy.get_param('~point_step', 1))
        self.min_range = float(rospy.get_param('~min_range', 0.12))
        self.max_range = float(rospy.get_param('~max_range', 12.0))
        angle_min_deg = rospy.get_param('~angle_min_deg', -30)
        angle_max_deg = rospy.get_param('~angle_max_deg', 30)
        self.angle_min_limit = np.deg2rad(float(angle_min_deg)) if angle_min_deg is not None else None
        self.angle_max_limit = np.deg2rad(float(angle_max_deg)) if angle_max_deg is not None else None
        self.draw_radius = int(rospy.get_param('~draw_radius', 2))
        self.draw_thickness = int(rospy.get_param('~draw_thickness', -1))
        bgr = rospy.get_param('~draw_bgr', '0,255,255')
        self.draw_color = tuple(int(c) for c in bgr.split(',')) # B,G,R


        self.bridge = CvBridge()
        self.scan_msg = None
        self.load_calibration(self.calib_yaml)


        self.sub_img = rospy.Subscriber(self.image_topic, Image, self.cb_image, queue_size=1)
        self.sub_lid = rospy.Subscriber(self.scan_topic, LaserScan, self.cb_scan, queue_size=1)
        self.pub_img = rospy.Publisher('lidar_on_image', Image, queue_size=1)


        if self.angle_min_limit is not None or self.angle_max_limit is not None:
            amin = np.rad2deg(self.angle_min_limit) if self.angle_min_limit is not None else None
            amax = np.rad2deg(self.angle_max_limit) if self.angle_max_limit is not None else None
            rospy.loginfo('Overlay ROI angles: min=%s°, max=%s°', str(amin), str(amax))
        rospy.loginfo('Overlay node ready: projecting LiDAR points onto camera image')


    def load_calibration(self, path):
        with open(path, 'r') as f:
            cfg = yaml.safe_load(f)
        self.img_w = int(cfg['image']['width'])
        self.img_h = int(cfg['image']['height'])
        self.rectified = bool(cfg['image'].get('rectified', True))


        K = np.array(cfg['camera_intrinsics']['K'], dtype=np.float64).reshape(3,3)
        D = np.array(cfg['camera_intrinsics'].get('D', [0,0,0,0,0]), dtype=np.float64).ravel()
        self.K = K
        self.D = D if not self.rectified else np.zeros_like(D)


        T = np.array(cfg['extrinsics']['T_lidar_camera'], dtype=np.float64).reshape(4,4)
        self.R = T[:3,:3]
        self.t = T[:3, 3:4]
        self.clamp = bool(cfg.get('viz', {}).get('clamp_to_image', True))


    def cb_scan(self, msg):
        self.scan_msg = msg


    def project_points(self, ranges, angle_min, angle_inc, rmin, rmax):
        # 수평 2D LiDAR → (x,y,0) → 카메라 좌표 → 픽셀
        us, vs = [], []
        for i in range(0, len(ranges), self.point_step):
            r = ranges[i]
            if not np.isfinite(r) or r <= rmin or r >= rmax:
                continue
            theta = angle_min + i * angle_inc
            if self.angle_min_limit is not None and theta < self.angle_min_limit:
                continue
            if self.angle_max_limit is not None and theta > self.angle_max_limit:
                continue
            xl = r * np.cos(theta)
            yl = r * np.sin(theta)
            Xl = np.array([[xl],[yl],[0.0]])
            Xc = self.R.dot(Xl) + self.t # (3,1)
            X, Y, Z = Xc[0,0], Xc[1,0], Xc[2,0]
            if Z <= 0:
                continue
            uvw = self.K.dot(np.array([X, Y, Z]))
            u = uvw[0]/uvw[2]
            v = uvw[1]/uvw[2]
            if self.clamp:
                if 0 <= u < self.img_w and 0 <= v < self.img_h:
                    us.append(u); vs.append(v)
            else:
                us.append(u); vs.append(v)
        return us, vs


    def cb_image(self, msg):
        if self.scan_msg is None:
            return
        # 최신 이미지 가져와 OpenCV로
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # LiDAR 포인트 투영
        s = self.scan_msg
        us, vs = self.project_points(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max)
        # 그리기
        for u, v in zip(us, vs):
            cv2.circle(img, (int(round(u)), int(round(v))), self.draw_radius, self.draw_color, self.draw_thickness, lineType=cv2.LINE_AA)
        # 퍼블리시
        cv2.imshow("lidar_camera_cali", img)
        cv2.waitKey(1)
        out_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out_msg.header = msg.header # 원본 타임스탬프/프레임 유지
        self.pub_img.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('overlay_lidar_on_image')
    LidarToImageOverlay()
    rospy.spin()
