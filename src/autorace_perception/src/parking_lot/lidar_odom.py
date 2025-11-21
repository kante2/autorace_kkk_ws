#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

class LidarOdom(object):
    def __init__(self):
        rospy.init_node("lidar_odom_from_tf")

        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.pub_topic  = rospy.get_param("~odom_topic", "/odom")

        self.odom_pub = rospy.Publisher(self.pub_topic, Odometry, queue_size=1)
        self.tf_listener = tf.TransformListener()

        self.prev_time = None
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None

        rospy.Timer(rospy.Duration(0.02), self.timer_callback)  # 50 Hz 정도

        rospy.loginfo("[lidar_odom_from_tf] started, using tf %s -> %s",
                      self.odom_frame, self.base_frame)

    def timer_callback(self, event):
        try:
            now = rospy.Time(0)
            (trans, rot) = self.tf_listener.lookupTransform(
                self.odom_frame, self.base_frame, now
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        x = trans[0]
        y = trans[1]

        # quaternion -> yaw
        import tf.transformations as tft
        roll, pitch, yaw = tft.euler_from_quaternion(rot)

        t = rospy.Time.now()

        vx = 0.0
        vy = 0.0
        wz = 0.0

        if self.prev_time is not None:
            dt = (t - self.prev_time).to_sec()
            if dt > 1e-3:
                vx = (x - self.prev_x) / dt
                vy = (y - self.prev_y) / dt
                dyaw = (yaw - self.prev_yaw)
                # 각도 wrap 보정
                while dyaw > 3.14159:
                    dyaw -= 2.0 * 3.14159
                while dyaw < -3.14159:
                    dyaw += 2.0 * 3.14159
                wz = dyaw / dt

        self.prev_time = t
        self.prev_x = x
        self.prev_y = y
        self.prev_yaw = yaw

        odom = Odometry()
        odom.header.stamp = t
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = rot[0]
        odom.pose.pose.orientation.y = rot[1]
        odom.pose.pose.orientation.z = rot[2]
        odom.pose.pose.orientation.w = rot[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

if __name__ == "__main__":
    try:
        node = LidarOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
