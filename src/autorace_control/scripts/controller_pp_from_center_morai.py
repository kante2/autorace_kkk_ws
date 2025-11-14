#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys, time, math
from std_msgs.msg import Float32
from morai_msgs.msg import CtrlCmd

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

class LaneCenterController:
    def __init__(self):
        rospy.init_node("lane_center_controller")
        rospy.loginfo("lane_center_controller (dx_px -> morai_msgs/CtrlCmd)")

        # --- Topics ---
        self.topic_dx_px   = rospy.get_param("~topic_dx_px", "/perception/dx_px")
        self.topic_ctrlcmd = rospy.get_param("~topic_ctrl_cmd", "/ctrl_cmd")

        sub_resolved = rospy.resolve_name(self.topic_dx_px)
        pub_resolved = rospy.resolve_name(self.topic_ctrlcmd)
        rospy.loginfo(f"[lane_ctrl] subscribe='{sub_resolved}', publish='{pub_resolved}'")

        # --- MORAI Cmd Params ---
        self.longlCmdType  = int(rospy.get_param("~longlCmdType", 2))
        self.max_steer_rad = float(rospy.get_param("~max_steer_rad", 0.5))
        self.steer_sign    = float(rospy.get_param("~steer_sign", 1.0))  # 좌양 +rad면 1.0, 반대면 -1.0

        # --- Control Params ---
        self.max_abs_dx_px = float(rospy.get_param("~max_abs_dx_px", 100.0))
        self.dx_tolerance  = float(rospy.get_param("~dx_tolerance", 3.0))
        self.steer_gain    = float(rospy.get_param("~steer_gain", 1.0))
        self.alpha_ema     = float(rospy.get_param("~steer_smoothing_alpha", 0.2))
        self.max_delta     = float(rospy.get_param("~max_steer_delta_per_cycle", 0.08))

        # 속도 계획
        self.base_speed_mps = float(rospy.get_param("~base_speed_mps", 7.0))
        self.min_speed_mps  = float(rospy.get_param("~min_speed_mps", 0.8))
        self.speed_drop_gain= float(rospy.get_param("~speed_drop_gain", 0.5))

        # 안전 타임아웃
        self.dx_timeout_sec = float(rospy.get_param("~dx_timeout_sec", 1.0))

        # --- Publisher/State ---
        self.pub_ctrl   = rospy.Publisher(self.topic_ctrlcmd, CtrlCmd, queue_size=10)
        self.prev_steer = 0.0
        self._last_cb_t = None

        self._latest_steer_cmd = 0.0
        self._latest_speed_cmd = self.base_speed_mps

        rospy.Subscriber(self.topic_dx_px, Float32, self.CB_dx, queue_size=20)

        self._cmd_template = CtrlCmd()
        self._cmd_template.longlCmdType = self.longlCmdType
        self._cmd_template.accel = 0.0
        self._cmd_template.brake = 0.0

    def CB_dx(self, msg: Float32):
        self._last_cb_t = time.time()
        dx = float(msg.data)

        if abs(dx) <= self.dx_tolerance:
            err_norm = 0.0
        else:
            err_norm = clamp(dx / self.max_abs_dx_px, -1.0, 1.0)

        steer_raw = math.tanh(self.steer_gain * err_norm)
        steer_smooth = self.alpha_ema * steer_raw + (1.0 - self.alpha_ema) * self.prev_steer
        delta = clamp(steer_smooth - self.prev_steer, -self.max_delta, self.max_delta)
        steer_cmd = self.prev_steer + delta
        self.prev_steer = steer_cmd

        speed_cmd = clamp(self.base_speed_mps - self.speed_drop_gain * abs(err_norm),
                          self.min_speed_mps, self.base_speed_mps)

        self._latest_steer_cmd = steer_cmd
        self._latest_speed_cmd = speed_cmd

        print(f"[lane_ctrl][CB] dx={dx:.1f}px  errN={err_norm:.3f}  steer={steer_cmd:.3f}  v={speed_cmd:.2f}")
        sys.stdout.flush()
        rospy.loginfo_throttle(0.5, f"[lane_ctrl][CB] dx={dx:.1f}px errN={err_norm:.3f} steer={steer_cmd:.3f} v={speed_cmd:.2f}")

    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            now = time.time()
            have_dx = (self._last_cb_t is not None) and ((now - self._last_cb_t) <= self.dx_timeout_sec)

            if have_dx:
                steer_cmd = self._latest_steer_cmd
                speed_cmd = self._latest_speed_cmd
            else:
                steer_cmd = 0.0
                speed_cmd = 0.0
                rospy.loginfo_throttle(1.0, "[lane_ctrl] waiting /perception/dx_px ... (timeout)")

            # ★★★ 부호 반전: 바로 여기 한 줄만 기존에서 반대로 ★★★
            steering_rad = (-self.steer_sign) * steer_cmd * self.max_steer_rad

            cmd = CtrlCmd()
            cmd.longlCmdType = self._cmd_template.longlCmdType
            cmd.steering     = float(steering_rad)
            cmd.velocity     = float(speed_cmd)
            cmd.accel        = self._cmd_template.accel
            cmd.brake        = self._cmd_template.brake

            self.pub_ctrl.publish(cmd)
            rospy.loginfo_throttle(0.5, f"[lane_ctrl][loop] have_dx={have_dx} steer={steer_cmd:.3f} rad={steering_rad:.3f} v={speed_cmd:.2f}")
            rate.sleep()

if __name__ == "__main__":
    try:
        node = LaneCenterController()
        node.run()
    except rospy.ROSInterruptException:
        pass
