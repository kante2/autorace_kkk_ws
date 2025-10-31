#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, os
import numpy as np
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from tf.transformations import euler_from_quaternion


class stanly_controller :
    def __init__(self):
        #subscribe and publish
        rospy.init_node('stanly_controller', anonymous=True)
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("Ego_topic", EgoVehicleStatus, self.status_callback)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd',CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2
        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.forward_point = Point()
        self.current_postion = Point()
        self.target_vel = 30.0
        self.current_vel = 0.0
        self.is_look_forward_point = False
        self.vehicle_length = 3
        self.k = 0.8 #stanley constant
        self.v_t = 1 #vel constant to get crosstrack error
        self.prev_steering_angle = 0

        rate = rospy.Rate(15) # 15hz
        while not rospy.is_shutdown():
            if self.is_path ==True and self.is_odom==True and self.is_status :
                vehicle_position=self.current_postion
                self.is_look_forward_point= False
                translation=[vehicle_position.x, vehicle_position.y]
                #translation matrix global to local(vehicle)
                t=np.array([
                        [cos(self.vehicle_yaw), -sin(self.vehicle_yaw),translation[0]],
                        [sin(self.vehicle_yaw),cos(self.vehicle_yaw),translation[1]],
                        [0                    ,0                    ,1            ]])
                det_t=np.array([
                       [t[0][0],t[1][0],-(t[0][0]*translation[0]+t[1][0]*translation[1])],
                       [t[0][1],t[1][1],-(t[0][1]*translation[0]+t[1][1]*translation[1])],
                       [0      ,0      ,1                                               ]])
                temp = np.zeros((2,2)) #for save local point of distant minimum path point
                dis_min = 10000
                j = 0
                #roop all path to get distance minimun path point from vehicle
                for num,i in enumerate(self.path.poses) :
                    path_point=i.pose.position
                    global_path_point=[path_point.x,path_point.y,1]
                    local_path_point=det_t.dot(global_path_point)
                    if local_path_point[0] < 0:
                        continue
                    dis = sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                    if dis <= dis_min :
                        dis_min = dis
                        j = num
                        temp[0][0] = local_path_point[0]
                        temp[0][1] = local_path_point[1]
                        temp_global = global_path_point
                        self.is_look_forward_point = True
                    #path point to get path yaw
                    if num == j + 1 :
                        temp[1][0] = local_path_point[0]
                        temp[1][1] = local_path_point[1]
                if self.is_look_forward_point :
                    #get yaw_term through distance minimum path point
                    heading_error = atan2(temp[1][1] - temp[0][1], temp[1][0] - temp[0][0])
                    #get cross track error through distance minimum path point
                    cte = sin(self.vehicle_yaw)*(temp_global[0] - vehicle_position.x) - cos(self.vehicle_yaw)*(temp_global[1] - vehicle_position.y)
                    crosstrack_error = -atan2(self.k * cte, self.current_vel + self.v_t)

                    # get steering angle with heading_error anf crosstrack_error
                    steering_angle = heading_error + crosstrack_error
                    
                    #clip steering angle between (-30, 30) degree
                    steering_angle = np.clip(steering_angle, -pi/6, pi/6)
                    self.ctrl_cmd_msg.steering = steering_angle
                    self.ctrl_cmd_msg.velocity = self.target_vel
                    os.system('clear')
                    print("-------------------------------------")
                    print(" steering (deg) = ", self.ctrl_cmd_msg.steering * 180 / pi)
                    print(" velocity (kph) = ", self.ctrl_cmd_msg.velocity)
                    print("-------------------------------------")
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                else :
                    os.system('clear')
                    print("can't find local_forward_point")
            else :
                os.system('clear')
                if not self.is_path:
                    print("[1] can't subscribe '/local_path' topic...")
                if not self.is_odom:
                    print("[2] can't subscribe '/odom' topic...")
                if not self.is_status:
                    print("[3] can't subscribe '/Ego_topic' topic")
            self.is_path = self.is_odom = False
            rate.sleep()
            
    #callback function for subscribe
    def status_callback(self, msg):
        self.is_status = True
        self.current_vel = msg.velocity.x
    def path_callback(self,msg):
        self.is_path=True
        self.path=msg
    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y


if __name__ == '__main__':
    try:
        test_track=stanly_controller()
    except rospy.ROSInterruptException:
        pass