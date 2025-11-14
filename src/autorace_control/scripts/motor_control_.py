#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import json
import rospy
from std_msgs.msg import Float64, Float32MultiArray, String

class PPFromCurvatureNode:
    def __init__(self):
        rospy.init_node("pp_from_curvature")
        rospy.loginfo("[pp_from_curvature_node started]")

        # === Params ===
        self.curv_topic   = rospy.get_param("~curvature_topic", "/perception/lane_curvature_m")
        self.curv_source  = rospy.get_param("~curvature_source", "average")  # average|left|right|minabs
        self.wheelbase    = rospy.get_param("~wheelbase", 0.26)  # m (차 축간 거리)
        self.max_steer    = rospy.get_param("~max_steer_rad", 0.5)  # rad, 물리 조향 한계
        self.servo_center = rospy.get_param("~servo_center", 0.5)   # 서보 중립값
        self.servo_scale  = rospy.get_param("~servo_scale", 0.5)    # δ=max일 때 center±scale 로 보냄
        self.v_min        = rospy.get_param("~v_min", 1000)          # m/s
        self.v_max        = rospy.get_param("~v_max", 3000)          # m/s
        self.ay_max       = rospy.get_param("~ay_max", 1.5)         # m/s^2 측가속 한계
        self.kappa_eps    = rospy.get_param("~kappa_eps", 1e-6)     # 0 나눔 방지
        self.alpha_kappa  = rospy.get_param("~alpha_kappa", 0.6)    # 곡률 EMA
        self.alpha_cmd    = rospy.get_param("~alpha_cmd", 0.4)      # 명령 EMA
        self.fail_straight= rospy.get_param("~fail_straight_speed", 0.8)  # κ없을 때 속도
        self.timeout_s    = rospy.get_param("~timeout_s", 0.5)

        # === State ===
        self.kappa_f      = 0.0
        self.last_msg_t   = rospy.Time(0)
        self.steer_cmd_f  = self.servo_center
        self.speed_cmd_f  = 0.0

        # === Publishers ===
        self.pub_servo = rospy.Publisher("/commands/servo/position", Float64, queue_size=10)
        self.pub_speed = rospy.Publisher("/commands/motor/speed", Float64, queue_size=10)
        self.pub_debug = rospy.Publisher("/pp_from_curv/debug", String, queue_size=10)

        # === Subscriber ===
        rospy.Subscriber(self.curv_topic, Float32MultiArray, self.CB_curvature, queue_size=10)

        rospy.Timer(rospy.Duration(0.02), self.on_timer)  # 50 Hz 출력

    def pick_kappa(self, k_left, k_right):
        # 선택 정책
        vals = []
        if k_left is not None and not math.isnan(k_left): vals.append(('L', k_left))
        if k_right is not None and not math.isnan(k_right): vals.append(('R', k_right))
        if not vals:
            return None, "none"
        if self.curv_source == "left":
            chosen = ('L', vals[0][1]) if vals[0][0]=='L' else ( 'L', None )
            return chosen[1], "left"
        if self.curv_source == "right":
            for tag,v in vals:
                if tag=='R': return v, "right"
            return None, "right"
        if self.curv_source == "minabs":
            tag,v = min(vals, key=lambda tv: abs(tv[1]))
            return v, "minabs"
        # default: average
        if len(vals)==2:
            return 0.5*(vals[0][1]+vals[1][1]), "avg2"
        return vals[0][1], "single"

    def ema(self, prev, new, alpha):
        return alpha*prev + (1.0-alpha)*new

    def CB_curvature(self, msg: Float32MultiArray):
        self.last_msg_t = rospy.Time.now()
        # msg.data = [left, right]
        k_left  = msg.data[0] if len(msg.data) > 0 else float('nan')
        k_right = msg.data[1] if len(msg.data) > 1 else float('nan')
        kappa, mode = self.pick_kappa(k_left, k_right)

        if kappa is None:
            return

        # 저역통과 필터
        self.kappa_f = self.ema(self.kappa_f, kappa, self.alpha_kappa)

        # 디버그
        dbg = {
            "k_left": k_left, "k_right": k_right,
            "kappa_raw": kappa, "kappa_f": self.kappa_f, "mode": mode
        }
        self.pub_debug.publish(String(data=json.dumps(dbg)))

    # *** 조향/속도 명령 계산
    def compute_commands(self, kappa):
        # 1) 조향: δ = atan(L * κ)
        delta = math.atan(self.wheelbase * kappa)
        delta = max(-self.max_steer, min(self.max_steer, delta))
        # 서보 맵핑: max 조향에서 center±servo_scale
        servo = self.servo_center + (delta / self.max_steer) * self.servo_scale

        # 2) 속도: a_y = v^2 * κ <= a_y_max  => v <= sqrt(a_y_max / |κ|)
        k_abs = max(abs(kappa), self.kappa_eps)
        v_lim = math.sqrt(self.ay_max / k_abs)
        v_cmd = min(self.v_max, max(self.v_min, v_lim))
        return servo, v_cmd, delta, v_lim

    def on_timer(self, _evt):
        now = rospy.Time.now()
        # 타임아웃/데이터 없음: 직진 & 안전 속도
        if (now - self.last_msg_t).to_sec() > self.timeout_s:
            target_servo = self.servo_center
            target_speed = self.fail_straight
            delta = 0.0
            v_lim = self.v_max
        else:
            target_servo, target_speed, delta, v_lim = self.compute_commands(self.kappa_f)

        # 명령 EMA로 부드럽게
        self.steer_cmd_f = self.ema(self.steer_cmd_f, target_servo, self.alpha_cmd) # 1000이면 약간 느리게 감,
        self.speed_cmd_f = self.ema(self.speed_cmd_f, target_speed, self.alpha_cmd)

        self.pub_servo.publish(Float64(self.steer_cmd_f))
        self.pub_speed.publish(Float64(self.speed_cmd_f))

        # 주기 디버그(1Hz)
        if int(now.to_sec()) % 1 == 0:
            dbg = {
                "servo_cmd": self.steer_cmd_f,
                "speed_cmd_mps": self.speed_cmd_f,
                "delta_rad": delta,
                "kappa_f": self.kappa_f,
                "v_lim": v_lim
            }
            self.pub_debug.publish(String(data=json.dumps(dbg)))

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    node = PPFromCurvatureNode()
    node.spin()
