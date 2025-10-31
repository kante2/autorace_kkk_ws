#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from math import pi
from nav_msgs.msg import Odometry



class Ego_listener():
    def __init__(self):
        rospy.init_node('status_listener', anonymous=True)
        
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.spin()


    def odom_callback(self,msg):
        self.is_odom = True

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.orientation_x = msg.pose.pose.orientation.x
        self.orientation_y = msg.pose.pose.orientation.y
        self.orientation_z = msg.pose.pose.orientation.z
        self.orientation_w = msg.pose.pose.orientation.w

        br = tf.TransformBroadcaster()
        br.sendTransform((self.x, self.y, 1),
                        (self.orientation_x,self.orientation_y,self.orientation_z,self.orientation_w),
                        rospy.Time.now(),
                        "Ego",
                        "map")

if __name__ == '__main__':
    try:
        tl=Ego_listener()
    except rospy.ROSInternalException:
        pass
