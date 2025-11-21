#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy 
from math import *
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, PoseStamped
from std_msgs.msg import Float64
import numpy as np
import tf2_ros
from lidar_preprocessing import preprocess_lidar
from parking_lot_detection import parking_detect
from goal_pose_pub import compute_goal_from_lines, publish_parking_goal

class Parking_mission:
    def __init__(self):
        rospy.init_node("parking_mission")
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        self.scan_msg = None
        self.preprocess_lidar = preprocess_lidar
        self.parking_detect = parking_detect
        
        # TEB로 goal 보내는 publisher
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal",  # 기본 move_base/TEB가 구독하는 PoseStamped 토픽
            PoseStamped,
            queue_size=1
        )
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.goal_published = False
        self.last_goal_msg = None  # 계속 퍼블리시할 목표 캐시

        self.timer = rospy.Timer(rospy.Duration(0.1), self.processing)  # 10 Hz processing loop
        

    def scan_callback(self, scan_msg):
        self.scan_msg = scan_msg
    
    def processing(self, event):
        # 이미 goal을 보냈으면 같은 목표를 반복 퍼블리시해 TEB가 놓치지 않도록 함
        if self.goal_published and self.last_goal_msg is not None:
            self.goal_pub.publish(self.last_goal_msg)
            return

        if self.scan_msg is None:
            return
        angle_ranges, dist_ranges = self.preprocess_lidar(self.scan_msg)
        
        #print(len(angle_ranges)/2+1)
        #print(f"angle:{angle_ranges[0]} dist_ranges:{dist_ranges[0]}")
        #print(f"angle:{angle_ranges[160]} dist_ranges:{dist_ranges[160]}")
        #print(f"angle:{angle_ranges[-1]} dist_ranges:{dist_ranges[-1]}")

        #print(f"angle_ranges:{angle_ranges}")
        #print(f"dist_ranges:{dist_ranges}") 

        is_u_shape, lines = self.parking_detect(angle_ranges, dist_ranges) 
        if not is_u_shape:
            print("No parking")
        else:
            print("U-shape parking structure detected")

        # 이미 goal 보냈으면 또 안 보냄
        if self.goal_published:
            return

        # lines로부터 goal pose 계산
        success, gx, gy, gyaw = compute_goal_from_lines(
            lines,
            min_width=0.3,   # 필요하면 파라미터화
            min_depth=0.2,
            wall_offset=0.1
        )

        if not success:
            print("U-shape is detected but not enough width/depth")
            return

        # goal pose publish
        adjusted_yaw, published_msg = publish_parking_goal(
            self.goal_pub,
            gx,
            gy,
            gyaw,
            frame_id="base_link",        # 라이다/로봇 기준으로 계산된 좌표
            target_frame="odom",         # 내비게이션이 필요로 하는 odom 프레임으로 변환
            tf_buffer=self.tf_buffer
        )

        if published_msg is None:
            rospy.logwarn("goal 변환/퍼블리시 실패: tf 준비가 안 됐을 수 있습니다. 다음 루프에서 재시도합니다.")
            return

        rospy.loginfo(
            "Parking goal published: base_link (x=%.2f, y=%.2f, yaw=%.1f deg), target frame=%s",
            gx, gy, degrees(adjusted_yaw), published_msg.header.frame_id
        )

        self.goal_published = True
        self.last_goal_msg = published_msg

def main():
    try:
        parking_mission = Parking_mission()
        rospy.spin()
    except rospy.ROSInternalException:
        pass

if __name__ == '__main__':
    main()    
