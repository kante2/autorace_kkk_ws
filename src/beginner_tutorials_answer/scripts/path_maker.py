#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from re import I
import rospy
import rospkg
from math import sqrt
from turtlesim.msg import Pose


class pathMaker :
    
    def __init__(self, pkg_name, path_name):
        rospy.init_node('path_maker', anonymous=True)
        # listen to /turtle1/pose 
        rospy.Subscriber("/turtle1/pose", Pose, self.status_callback)

        self.prev_x = 0
        self.prev_y = 0
        self.is_status=False
        rospack = rospkg.RosPack()
        pkg_path=rospack.get_path(pkg_name)
        full_path=pkg_path + '/'+path_name+'.txt'
        self.f=open(full_path, 'w')

        while not rospy.is_shutdown():
            if self.is_status==True :
                self.path_make()
        self.f.close()

    def path_make(self):
        x=self.status_msg.x
        y=self.status_msg.y
        distance=sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
        if distance >0.3:
            data='{0}\t{1}\n'.format(x,y)
            self.f.write(data)
            self.prev_x=x
            self.prev_y=y
            print("write : ", x,y)
    
    def status_callback(self,msg):
        self.is_status=True
        self.status_msg=msg

if __name__ == '__main__' :
    try:
        p_m=pathMaker("beginner_tutorials", "turtle_path")
    except rospy.ROSInternalException:
        pass
            