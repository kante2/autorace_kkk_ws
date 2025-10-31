#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf
from turtlesim.msg import Pose

class turtle_listener():
    def __init__(self):
        rospy.init_node('status_listener', anonymous=True)
        rospy.Subscriber('/turtle1/pose', Pose, self.statusCB)
        self.status_msg=Pose()
        rospy.spin()

    def statusCB(self,data): ## turtle Status subscriber
        self.status_msg=data
        print("tf broad cast")
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.x, self.status_msg.y, 0),
                        tf.transformations.quaternion_from_euler(0,0, self.status_msg.theta),
                        rospy.Time.now(),
                        "turtle",
                        "map")

if __name__ == '__main__':
    try:
        tl=turtle_listener()
    except rospy.ROSInternalException:
        pass
