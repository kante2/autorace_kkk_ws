
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, sys, math, yaml
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

"""
ROS params:
  ~image_topic     : camera image topic (default: /usb_cam/image_raw)
  ~scan_topic      : lidar scan topic (default: /scan)
  ~camera_yaml     : OpenCV/CameraInfo yaml (K,D) for optional undistort
  ~use_undistort   : undistort clicked pixel before saving (default: true)
  ~output_dir      : directory to save files (default: ./data)
  ~min_range       : override scan.range_min (optional)
  ~max_range       : override scan.range_max (optional)

Keys:
  Mouse Left  : choose pixel (u,v)
  SPACE       : save one pair to data.txt  (format: x y u v)
  r           : reset clicked pixel
  q or ESC    : quit
"""

class CorrSaver:
    def __init__(self):
        self.image_topic   = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.scan_topic    = rospy.get_param("~scan_topic", "/scan")
        self.camera_yaml   = rospy.get_param("~camera_yaml", "")
        self.use_undist    = rospy.get_param("~use_undistort", True)
        self.output_dir    = rospy.get_param("~output_dir", "data")
        self.override_minr = rospy.get_param("~min_range", None)
        self.override_maxr = rospy.get_param("~max_range", None)

        os.makedirs(self.output_dir, exist_ok=True)
        self.f_pair = open(os.path.join(self.output_dir, "data.txt"), "a")

        # 카메라 파라미터(언디스토트용, 없으면 패스)
        self.K = None; self.D = None
        if self.camera_yaml:
            with open(self.camera_yaml, 'r') as f:
                cam = yaml.safe_load(f)
            K = np.array(cam['camera_matrix']['data'], dtype=np.float64).reshape(3,3)
            d = cam['distortion_coefficients']['data']
            if len(d) < 5: d += [0.0]*(5-len(d))
            D = np.array(d[:5], dtype=np.float64).reshape(1,5)
            self.K, self.D = K, D

        if self.use_undist and (self.K is None or self.D is None):
            rospy.logwarn("use_undistort=true 이지만 camera_yaml이 없거나 K/D 로드 실패 → 언디스토트 비활성화")
            self.use_undist = False

        self.bridge = CvBridge()
        self.last_img  = None
        self.last_scan = None
        self.click_uv  = None

        self.img_sub  = rospy.Subscriber(self.image_topic, Image, self.cb_img, queue_size=1)
        self.scan_sub = rospy.Subscriber(self.scan_topic,  LaserScan, self.cb_scan, queue_size=1)

        self.win = "Save correspondences | SPACE=save  r=reset  q=quit"
        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.win, self.on_mouse)
        rospy.loginfo("Ready. 화면에서 타깃 중심 클릭 후 SPACE를 눌러 (x y u v) 저장하세요.")

    def cb_img(self, msg):
        try:
            self.last_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("cv_bridge: %s", str(e))

    def cb_scan(self, msg):
        self.last_scan = msg

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_uv = (int(x), int(y))

    def undistort_pixel(self, uv):
        if not self.use_undist: return uv
        pts = np.array(uv, dtype=np.float64).reshape(-1,1,2)
        und = cv2.undistortPoints(pts, self.K, self.D, P=self.K).reshape(-1,2)
        return (float(und[0,0]), float(und[0,1]))

    def pick_nearest_xy(self, scan):
        rmin = float('inf'); idx = -1
        r_min = self.override_minr if self.override_minr is not None else scan.range_min
        r_max = self.override_maxr if self.override_maxr is not None else scan.range_max
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r): continue
            if r < r_min or r > r_max: continue
            if r < rmin:
                rmin = r; idx = i
        if idx < 0:
            return None
        theta = scan.angle_min + idx * scan.angle_increment
        x = rmin * math.cos(theta); y = rmin * math.sin(theta)
        return (x, y)

    def draw_hud(self, img):
        cv2.putText(img, "Click target, SPACE=save  r=reset  q=quit",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)
        cv2.putText(img, f"undistort={'ON' if self.use_undist else 'OFF'}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)
        if self.click_uv is not None:
            cv2.circle(img, self.click_uv, 4, (0,255,255), -1, cv2.LINE_AA)

    def spin(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.last_img is not None:
                vis = self.last_img.copy()
                self.draw_hud(vis)
                cv2.imshow(self.win, vis)
                key = cv2.waitKey(1) & 0xFF

                if key in (ord('q'), 27):  # q / ESC
                    break
                elif key == ord('r'):
                    self.click_uv = None
                elif key == 32:  # SPACE
                    if self.last_scan is None or self.click_uv is None:
                        rospy.logwarn("스캔과 클릭 픽셀이 모두 필요합니다.")
                    else:
                        xy = self.pick_nearest_xy(self.last_scan)
                        if xy is None:
                            rospy.logwarn("유효한 LiDAR 리턴이 없습니다.")
                        else:
                            u,v = self.click_uv
                            if self.use_undist: u,v = self.undistort_pixel((u,v))
                            # 저장: x y u v
                            self.f_pair.write(f"{xy[0]:.9f} {xy[1]:.9f} {u:.3f} {v:.3f}\n")
                            self.f_pair.flush()
                            rospy.loginfo("Saved: x=%.3f y=%.3f  u=%.1f v=%.1f", xy[0], xy[1], u, v)
                            self.click_uv = None
            rate.sleep()

        self.f_pair.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("save_correspondences")
    CorrSaver().spin()

"""
rosrun get_pairpoints.py \
  _image_topic:=/usb_cam/image_raw \
  _scan_topic:=/scan \
  _camera_yaml:=$(rospack find camera_calibration)/config/cam_calibration.yaml \
  _use_undistort:=true \
  _output_dir:=/tmp/calib_pairs

"""