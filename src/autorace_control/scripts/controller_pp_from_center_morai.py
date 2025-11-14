#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped

class LaneCenterPurePursuitController:
    def __init__(self):
        rospy.init_node("lane_center_pp_controller")
        rospy.loginfo("lane_center_pp_controller started")

        # ------------------ Parameters (set in launch or rosparam) ------------------
        # 1) Vehicle / control
        self.wheelbase_m          = rospy.get_param("~wheelbase_m", 0.26)   # 차량 휠베이스
        self.max_steer_rad        = rospy.get_param("~max_steer_rad", 0.6)  # 스티어 제한
        self.v_straight_max       = rospy.get_param("~v_straight_max", 2.5) # 직선 최대 속도[m/s]
        self.a_lat_max            = rospy.get_param("~a_lat_max", 1.8)      # 허용 횡가속[m/s^2]
        self.speed_lpf_alpha      = rospy.get_param("~speed_lpf_alpha", 0.6)# 속도 저역통과

        # 2) Lookahead design
        self.ld_base              = rospy.get_param("~ld_base", 1.0)   # 기본 Ld [m]
        self.ld_min               = rospy.get_param("~ld_min", 0.6)
        self.ld_max               = rospy.get_param("~ld_max", 2.5)
        self.ld_gain_v            = rospy.get_param("~ld_gain_v", 0.25) # 속도 연동
        self.ld_gain_kappa        = rospy.get_param("~ld_gain_kappa", 0.0) # 곡률 연동(옵션)

        # 3) BEV(pixel) -> base_link(meter) 변환 파라미터
        #    - origin_px: BEV에서 차량 기준 원점에 대응하는 픽셀(보통 하단 중앙: (w/2, h-1))
        self.bev_origin_px_x      = rospy.get_param("~bev_origin_px_x", 320.0)
        self.bev_origin_px_y      = rospy.get_param("~bev_origin_px_y", 479.0)
        self.m_per_px_x           = rospy.get_param("~meters_per_px_x", 0.01)  # 오른쪽 1 px = 몇 m (→ 차량 좌측은 + 이므로 나중에 부호 반전)
        self.m_per_px_y           = rospy.get_param("~meters_per_px_y", 0.01)  # 아래쪽 1 px = 몇 m (→ 차량 전방은 + 이므로 나중에 부호 반전)

        # 4) 기타
        self.publish_debug_target = rospy.get_param("~publish_debug_target", True)

        # ------------------ State ------------------
        self._latest_center_px = None      # (cx_px, cy_px) in BEV pixels
        self._latest_kappa     = 0.0       # 중앙 곡률(없으면 0 가정)
        self._filtered_speed   = 0.0       # LPF된 목표 속도

        # ------------------ Publishers ------------------
        self.pub_steer = rospy.Publisher("/control/steering_cmd", Float32, queue_size=1)
        self.pub_speed = rospy.Publisher("/control/target_speed", Float32, queue_size=1)
        self.pub_ld_pt = rospy.Publisher("/control/lookahead_target", PointStamped, queue_size=1)

        # ------------------ Subscribers ------------------
        rospy.Subscriber("/perception/center_point_px", PointStamped, self._on_center_point_px, queue_size=5)
        rospy.Subscriber("/perception/curvature_center", Float32, self._on_center_curvature, queue_size=5)

        rospy.Timer(rospy.Duration(0.02), self._control_loop)  # 50 Hz

    # ------------------ Callbacks ------------------
    def _on_center_point_px(self, msg: PointStamped):
        # 입력은 BEV 픽셀 좌표(OpenCV: x 오른쪽+, y 아래+)
        self._latest_center_px = (msg.point.x, msg.point.y)

    def _on_center_curvature(self, msg: Float32):
        # 중앙 곡률(1/m). 퍼셉션에서 픽셀 기반이면, 퍼셉션 쪽에서 미터 변환해서 주는 걸 추천
        try:
            self._latest_kappa = float(msg.data)
        except Exception:
            self._latest_kappa = 0.0

    # ------------------ Geometry helpers ------------------
    def _bev_px_to_baselink_m(self, cx_px: float, cy_px: float):
        """
        BEV 픽셀 → 차량기준(base_link) 미터 좌표로 변환.
        - 픽셀 x: 오른쪽 +  → 차량 y: 왼쪽 +  ==> y_bl = -(cx_px - origin_x) * m_per_px_x
        - 픽셀 y: 아래쪽 +  → 차량 x: 전방 +  ==> x_bl = (origin_y - cy_px) * m_per_px_y
        """
        dx_px = cx_px - self.bev_origin_px_x
        dy_px = cy_px - self.bev_origin_px_y

        y_bl = -dx_px * self.m_per_px_x    # 오른쪽 픽셀 +는 차량 오른쪽이므로 부호 반전
        x_bl = -dy_px * self.m_per_px_y    # 아래 픽셀 +는 차량 뒤쪽이므로 부호 반전하여 전방+
        return x_bl, y_bl                   # (전방 X, 좌측 Y) in meters

    def _normalize(self, x: float, y: float):
        n = math.hypot(x, y)
        if n < 1e-6:
            return 1.0, 0.0, 1e-6
        return x / n, y / n, n

    # ------------------ Control core ------------------
    def _compute_dynamic_lookahead(self, current_speed_mps: float, kappa: float):
        Ld = self.ld_base + self.ld_gain_v * current_speed_mps - self.ld_gain_kappa * abs(kappa)
        return max(self.ld_min, min(self.ld_max, Ld))

    def _pure_pursuit_steering(self, lookahead_target_bl, Ld):
        """
        lookahead_target_bl: (tx, ty) in base_link [m]
        Pure Pursuit: kappa = 2*ty / Ld^2,  delta = atan(L * kappa)
        """
        tx, ty = lookahead_target_bl
        # 수치 안정: Ld가 너무 작아지면 보정
        Ld = max(Ld, 1e-3)
        kappa = 2.0 * ty / (Ld ** 2)
        steer = math.atan(self.wheelbase_m * kappa)
        # 제한
        steer = max(-self.max_steer_rad, min(self.max_steer_rad, steer))
        return steer, kappa

    def _speed_planner_from_curvature(self, kappa: float):
        """
        곡률 기반 속도 계획: v_curve = sqrt(a_lat_max / |kappa|).
        직선 상한과 저역통과 필터 적용.
        """
        kappa_abs = max(abs(kappa), 1e-5)
        v_curve = math.sqrt(self.a_lat_max / kappa_abs)
        v_cmd = min(self.v_straight_max, v_curve)
        # 1차 LPF
        self._filtered_speed = (self.speed_lpf_alpha * self._filtered_speed
                                + (1.0 - self.speed_lpf_alpha) * v_cmd)
        return self._filtered_speed

    # ------------------ Main loop ------------------
    def _control_loop(self, _evt):
        if self._latest_center_px is None:
            return

        # 1) BEV 픽셀 중앙점 → base_link 미터 좌표
        cx_px, cy_px = self._latest_center_px
        x_bl, y_bl = self._bev_px_to_baselink_m(cx_px, cy_px)

        # 2) 보라 벡터 = (x_bl, y_bl) 방향 단위벡터
        dir_x, dir_y, dist = self._normalize(x_bl, y_bl)

        # 3) 룩어헤드 (속도 기반 동적 or 정적)
        #    현재 속도 센서가 없다면, 필터 속도 또는 0으로 가정
        current_speed_est = self._filtered_speed
        Ld = self._compute_dynamic_lookahead(current_speed_est, self._latest_kappa)

        # 4) 가상 타깃점(웨이포인트): 방향벡터 * Ld
        target_bl = (dir_x * Ld, dir_y * Ld)

        # 5) 조향 계산 (Pure Pursuit)
        steer_cmd, kappa_cmd = self._pure_pursuit_steering(target_bl, Ld)

        # 6) 속도 계획 (곡률 기반)
        speed_cmd = self._speed_planner_from_curvature(kappa_cmd if abs(self._latest_kappa) < 1e-6 else self._latest_kappa)

        # 7) Publish
        self.pub_steer.publish(Float32(data=float(steer_cmd)))
        self.pub_speed.publish(Float32(data=float(speed_cmd)))

        if self.publish_debug_target:
            pt = PointStamped()
            pt.header.stamp = rospy.Time.now()
            pt.header.frame_id = "base_link"  # 차량 기준 좌표계
            pt.point.x = target_bl[0]
            pt.point.y = target_bl[1]
            pt.point.z = 0.0
            self.pub_ld_pt.publish(pt)

def main():
    node = LaneCenterPurePursuitController()
    rospy.spin()

if __name__ == "__main__":
    main()


# “기록된 경로”가 없을 땐 
# 보라 벡터(차선 중앙 방향)를 기준으로, 로봇 앞쪽으로 Ld 만큼 떨어진 점을 가상의 웨이포인트로 찍어서 
# Pure Pursuit 타깃으로 쓰도록 하는 제어기를 설계해보았다. - 1110 kt

