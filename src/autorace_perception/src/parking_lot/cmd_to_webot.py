#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class CmdVelToWebot(object):
    def __init__(self):
        rospy.init_node("cmdvel_to_f1tenth")

        self.wheelbase = rospy.get_param("~wheelbase", 0.26)
        self.use_steer_direct = rospy.get_param("~use_steer_direct", True)

        # servo parameter
        self.steer_center = rospy.get_param("~steer_center", 0.5)
        self.steer_gain   = rospy.get_param("~steer_gain", 1.0)
        self.steer_limit  = rospy.get_param("~steer_limit", 1.0)

        # input: teb /cmd_vel
        self.sub = rospy.Subscriber("/cmd_vel", Twist,
                                    self.cmdvel_callback, queue_size=1)

        latched = False
        self.motor_pub = rospy.Publisher("/commands/motor/speed",
                                         Float64, queue_size=1, latch=latched)
        self.servo_pub = rospy.Publisher("/commands/servo/position",
                                         Float64, queue_size=1, latch=latched)

        rospy.loginfo("[cmdvel_to_f1tenth] started")

    def cmdvel_callback(self, msg):
        v = msg.linear.x
        wz_or_delta = msg.angular.z

        # ================================
        # SPEED CONVERSION
        # ================================
        # 1000 → 0.26 m/s
        motor_gain = 1000.0 / 0.26
        motor_cmd = Float64()
        motor_cmd.data = v * motor_gain
        self.motor_pub.publish(motor_cmd)

        # ================================
        # STEERING CONVERSION
        # ================================
        if self.use_steer_direct:
            steer_angle = wz_or_delta
        else:
            if abs(v) < 1e-3:
                steer_angle = 0.0
            else:
                steer_angle = math.atan(self.wheelbase * wz_or_delta / v)

        servo_pos = self.steer_center + self.steer_gain * steer_angle
        servo_pos = max(-self.steer_limit, min(self.steer_limit, servo_pos))

        steer_cmd = Float64()
        steer_cmd.data = servo_pos
        self.servo_pub.publish(steer_cmd)

        rospy.loginfo_throttle(0.5,
            "[cmdvel] v=%.2f → motor=%.2f | steer=%.2f rad → servo=%.2f",
            v, motor_cmd.data, steer_angle, servo_pos
        )


if __name__ == "__main__":
    try:
        node = CmdVelToWebot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
