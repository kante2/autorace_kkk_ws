#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class CmdVelToWebot(object):
    def __init__(self):
        rospy.init_node("cmd_to_webot")

        self.wheelbase = rospy.get_param("~wheelbase", 0.26)
        self.use_steer_direct = rospy.get_param("~use_steer_direct", False)  # TEB면 False 권장

        # servo parameter
        self.steer_center = rospy.get_param("~steer_center", 0.5)
        self.steer_gain   = rospy.get_param("~steer_gain", 1.0)
        self.steer_limit  = rospy.get_param("~steer_limit", 1.0)
        self.max_steer_deg = rospy.get_param("~max_steer_deg", 20.0)  # ±20deg 기본
        self.servo_min = rospy.get_param("~servo_min", 0.0)
        self.servo_max = rospy.get_param("~servo_max", 1.0)

        # 전/후진 방향 전환 시 0을 한 번 거치기 위해 이전 모터 부호 기억
        self.prev_motor_sign = 0

        # input: teb /cmd_vel
        self.sub = rospy.Subscriber("/cmd_vel", Twist,
                                    self.cmdvel_callback, queue_size=1)

        latched = False
        self.motor_pub = rospy.Publisher("/commands/motor/speed",
                                         Float64, queue_size=1, latch=latched)
        self.servo_pub = rospy.Publisher("/commands/servo/position",
                                         Float64, queue_size=1, latch=latched)

        rospy.loginfo("[cmdvel_to_webot] started")

    def cmdvel_callback(self, msg):
        v = msg.linear.x
        wz_or_delta = msg.angular.z

        # 값이 너무 튀지 않게 속도 상한 (상황 보면서 조절)
        MAX_V = 0.7
        if v > MAX_V:
            v = MAX_V
        if v < -MAX_V:
            v = -MAX_V

        # ================================
        # SPEED CONVERSION
        # ================================
        # 1000 → 0.26 m/s
        motor_gain = 1000.0 / 0.26
        motor_cmd = Float64()

        if abs(v) < 1e-3:
            # 거의 0이면 그냥 정지
            desired_cmd = 0.0
        else:
            motor_raw = v * motor_gain

            # 최소 구동값 보장 (전/후진)
            if motor_raw >= 0.0:
                desired_cmd = max(motor_raw, 1000.0)
            else:
                desired_cmd = min(motor_raw, -1000.0)

        # 부호 판정
        desired_sign = 0 if abs(desired_cmd) < 1e-6 else (1 if desired_cmd > 0 else -1)

        # 부호가 바뀌면: 0을 먼저 한 번 퍼블리시한 뒤 새 명령을 보냄
        if self.prev_motor_sign != 0 and desired_sign != 0 and desired_sign != self.prev_motor_sign:
            stop_msg = Float64()
            stop_msg.data = 0.0
            self.motor_pub.publish(stop_msg)

        motor_cmd.data = desired_cmd

        # 셋째 자리 이하는 버림 (100 단위로 내림)
        motor_cmd.data = math.floor(motor_cmd.data / 100.0) * 100.0

        # 상한 클램핑 (혹시라도 너무 커지지 않도록)
        MAX_MOTOR = 4000.0
        if motor_cmd.data > MAX_MOTOR:
            motor_cmd.data = MAX_MOTOR
        if motor_cmd.data < -MAX_MOTOR:
            motor_cmd.data = -MAX_MOTOR

        self.motor_pub.publish(motor_cmd)
        self.prev_motor_sign = desired_sign

        # ================================
        # STEERING CONVERSION
        # ================================
        if self.use_steer_direct:
            # 이미 조향각(rad)을 받아온 경우만 True로
            steer_angle = wz_or_delta
        else:
            # /cmd_vel.angular.z = yaw rate [rad/s] 기준 (TEB 기본)
            if abs(v) < 1e-3:
                steer_angle = 0.0
            else:
                steer_angle = math.atan(self.wheelbase * wz_or_delta / v)

        # 스티어링 맵핑: 왼쪽(+rad)은 servo ↓, 오른쪽(-rad)는 servo ↑
        max_steer_rad = math.radians(self.max_steer_deg)
        servo_span = min(self.steer_center - self.servo_min,
                         self.servo_max - self.steer_center)
        servo_scale = servo_span / max_steer_rad if max_steer_rad > 1e-6 else 0.0

        servo_pos = self.steer_center - self.steer_gain * steer_angle * servo_scale
        servo_pos = max(self.servo_min, min(self.servo_max, servo_pos))

        steer_cmd = Float64()
        steer_cmd.data = servo_pos
        self.servo_pub.publish(steer_cmd)

        rospy.loginfo_throttle(
            0.5,
            "[cmdvel] v=%.2f → motor=%.1f | wz=%.2f → steer=%.2f rad → servo=%.2f",
            v, motor_cmd.data, wz_or_delta, steer_angle, servo_pos
        )


if __name__ == "__main__":
    try:
        node = CmdVelToWebot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
