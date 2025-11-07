#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class JoyToCmd(object):
    def __init__(self):
        # Topics
        self.joy_topic   = rospy.get_param("~joy_topic", "/joy")
        self.motor_topic = rospy.get_param("~motor_topic", "/commands/motor/speed")
        self.servo_topic = rospy.get_param("~servo_topic", "/commands/servo/position")

        # Axes
        self.speed_axis  = int(rospy.get_param("~speed_axis", 7))  # D-pad vertical
        self.steer_axis  = int(rospy.get_param("~steer_axis", 3))  # right stick vertical

        # Scales
        self.max_speed   = float(rospy.get_param("~max_speed", 2000.0))  # ±2000
        self.deadzone    = float(rospy.get_param("~deadzone", 0.05))     # 축 데드존
        self.publish_rate= float(rospy.get_param("~publish_rate", 50.0)) # Hz

        # State: last commands
        self.last_speed = 0.0
        self.last_servo = 0.5  # neutral

        # Publishers
        self.pub_speed = rospy.Publisher(self.motor_topic, Float64, queue_size=10)
        self.pub_servo = rospy.Publisher(self.servo_topic, Float64, queue_size=10)

        # Subscriber
        rospy.Subscriber(self.joy_topic, Joy, self.cb_joy)

        # Timer to continuously publish
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.on_timer)

        # Publish initial neutral
        self.pub_speed.publish(Float64(self.last_speed))
        self.pub_servo.publish(Float64(self.last_servo))

        rospy.loginfo("joy_to_cmd ready: joy=%s -> motor=%s, servo=%s (rate=%.1fHz)",
                      self.joy_topic, self.motor_topic, self.servo_topic, self.publish_rate)

    def apply_deadzone(self, v):
        return 0.0 if abs(v) < self.deadzone else v

    def cb_joy(self, msg):
        axes = msg.axes
        if not (0 <= self.speed_axis < len(axes)) or not (0 <= self.steer_axis < len(axes)):
            rospy.logwarn_throttle(1.0, "axes index out of range: len=%d, speed_axis=%d, steer_axis=%d",
                                   len(axes), self.speed_axis, self.steer_axis)
            return

        # Speed: axes[7] in [-1,1] -> [-max_speed, max_speed]
        raw_speed = self.apply_deadzone(axes[self.speed_axis])
        self.last_speed = clamp(self.max_speed * clamp(raw_speed, -1.0, 1.0),
                                -self.max_speed, self.max_speed)

        # Steering: map [-1,1] -> [0,1] with center 0.5
        raw_steer = axes[self.steer_axis]
        raw_steer = 0.0 if abs(raw_steer) < self.deadzone else raw_steer
        servo = (1.0 - raw_steer) * 0.57165  # 1->0.0, 0->0.5, -1->1.0
        self.last_servo = clamp(servo, 0.0, 1.0)

    def on_timer(self, _evt):
        # continuously publish last commands
        self.pub_speed.publish(Float64(self.last_speed))
        self.pub_servo.publish(Float64(self.last_servo))

def main():
    rospy.init_node("joy_to_cmd")
    JoyToCmd()
    rospy.spin()

if __name__ == "__main__":
    main()
