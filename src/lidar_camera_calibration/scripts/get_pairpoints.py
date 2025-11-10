#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, sys, math, yaml, rospy, cv2, numpy as np
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer as ATS

# ==LiDAR to Camera 사이의 대응 점 찾기==
"""
ROS params:
  ~sync_queue     : ATS queue size (default: 10)
  ~sync_slop_sec  : 허용 시간차(초) (default: 0.05 = 50 ms)
  ~image_topic : camera image topic (default: /usb_cam/image_raw) 
  ~scan_topic : lidar scan topic (default: /scan) 
  ~camera_yaml : OpenCV/CameraInfo yaml (K,D) for optional undistort 
  ~use_undistort : undistort clicked pixel before saving (default: false) 
  ~output_dir : directory to save files (default: ./data) 
  ~min_range : override scan.range_min (optional) 
  ~max_range : override scan.range_max (optional) 
  
Keys:
Mouse Left : choose pixel (u,v) 
SPACE : save one pair to data.txt (format: x y u v) 
r : reset clicked pixel 
q or ESC : quit


"""

class CorrSaver:
    def __init__(self):
        self.image_topic   = rospy.get_param("~image_topic", "/usb_cam/image_rect_color")
        self.scan_topic    = rospy.get_param("~scan_topic", "/scan")
        self.camera_yaml   = rospy.get_param("~camera_yaml", "")
        self.use_undist    = rospy.get_param("~use_undistort", False)   # rect면 False 권장
        self.output_dir    = rospy.get_param("~output_dir", "data")
        self.override_minr = rospy.get_param("~min_range", None)
        self.override_maxr = rospy.get_param("~max_range", None)
        self.sync_queue    = int(rospy.get_param("~sync_queue", 10))
        self.sync_slop     = float(rospy.get_param("~sync_slop_sec", 0.05))  # 50 ms

        os.makedirs(self.output_dir, exist_ok=True)
        self.f_pair = open(os.path.join(self.output_dir, "data.txt"), "a")

        # Camera params (optional undistort)
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
            rospy.logwarn("use_undistort=true 이지만 K/D 없음 → 언디스토트 비활성화")
            self.use_undist = False

        self.bridge = CvBridge()

        # 최신 "동기화된" 한 쌍(이미지+스캔) 저장
        self.sync_img_bgr = None
        self.sync_scan    = None
        self.sync_stamp   = None
        self.click_uv     = None
        self.last_dt_ms   = None   # 두 메시지 사이 시간차(ms)

        # --- message_filters 구독 + 동기화 ---
        self.img_sub  = Subscriber(self.image_topic, Image)
        self.scan_sub = Subscriber(self.scan_topic,  LaserScan)
        self.syncer   = ATS([self.img_sub, self.scan_sub],
                            queue_size=self.sync_queue,
                            slop=self.sync_slop)
        self.syncer.registerCallback(self.cb_sync)

        self.win = "Save correspondences (SYNC) | SPACE=save  r=reset  q=quit"
        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.win, self.on_mouse)

        rospy.loginfo("Ready (sync). rect이미지에서 타깃 클릭 후 SPACE로 (x y u v) 저장.")

    # 동기화 콜백: 항상 '같은 시각'의 쌍만 보관
    def cb_sync(self, img_msg, scan_msg):
        try:
            self.sync_img_bgr = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            self.sync_scan    = scan_msg
            # 시간차 로깅(디버그용)
            dt = (img_msg.header.stamp - scan_msg.header.stamp).to_sec()
            self.last_dt_ms = abs(dt*1000.0)
            self.sync_stamp = img_msg.header.stamp
        except Exception as e:
            rospy.logwarn("sync cv_bridge: %s", str(e))

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_uv = (int(x), int(y))

    def undistort_pixel(self, uv):
        if not self.use_undist: return uv
        pts = np.array(uv, dtype=np.float64).reshape(-1,1,2)
        und = cv2.undistortPoints(pts, self.K, self.D, P=self.K).reshape(-1,2)
        return (float(und[0,0]), float(und[0,1]))

    # 기본: "가장 가까운 r" 선택 (정확도↑ 위해 빔 인덱스 선택 기능을 추후 추가 권장)
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
        return (x, y, idx, rmin, theta)

    def draw_hud(self, img):
        cv2.putText(img, "Click target, SPACE=save  r=reset  q=quit",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)
        if self.last_dt_ms is not None:
            cv2.putText(img, f"sync Δt={self.last_dt_ms:.1f} ms",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)
        if self.click_uv is not None:
            cv2.circle(img, self.click_uv, 4, (0,255,255), -1, cv2.LINE_AA)

    def spin(self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            if self.sync_img_bgr is not None:
                vis = self.sync_img_bgr.copy()
                self.draw_hud(vis)
                cv2.imshow(self.win, vis)
                key = cv2.waitKey(1) & 0xFF

                if key in (ord('q'), 27):
                    break
                elif key == ord('r'):
                    self.click_uv = None
                elif key == 32:  # SPACE
                    if self.sync_scan is None or self.click_uv is None:
                        rospy.logwarn("동기화된 스캔과 클릭 픽셀이 모두 필요합니다.")
                    else:
                        res = self.pick_nearest_xy(self.sync_scan)
                        if res is None:
                            rospy.logwarn("유효한 LiDAR 리턴이 없습니다.")
                        else:
                            x, y, idx, r, th = res
                            u, v = self.click_uv
                            if self.use_undist:
                                u, v = self.undistort_pixel((u, v))
                            # 저장: x y u v (원하면 idx,r,theta,stamp도 함께)
                            self.f_pair.write(f"{x:.9f} {y:.9f} {u:.3f} {v:.3f}\n")
                            self.f_pair.flush()
                            rospy.loginfo("Saved: i=%d r=%.3f th=%.2fdeg | x=%.3f y=%.3f  u=%.1f v=%.1f",
                                          idx, r, math.degrees(th), x, y, u, v)
                            self.click_uv = None
            rate.sleep()

        self.f_pair.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("save_correspondences_sync")
    CorrSaver().spin()

"""
rosrun lidar_camera_calibration get_pairspoints.py \
  _image_topic:=/usb_cam/image_rect_color \
  _scan_topic:=/scan \
  _use_undistort:=false \
  _sync_queue:=10 \
  _sync_slop_sec:=0.05 \
  _output_dir:=/tmp/calib_pairs
"""

#rosbag 재생 시 /use_sim_time을 끄고 단일 머신 시간으로 재생하거나, 둘 다 /clock 기반으로 일치시키자.