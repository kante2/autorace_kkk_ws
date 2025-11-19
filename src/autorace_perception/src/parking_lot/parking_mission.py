import rospy 
from math import *
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, PoseStamped
from std_msgs.msg import Float64
import numpy as np
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
            "/move_base_simple/goal",  # TEB가 듣는 토픽에 맞게 바꿔도 됨
            PoseStamped,
            queue_size=1
        )
        self.goal_published = False

        self.timer = rospy.Timer(rospy.Duration(0.1), self.processing)  # 10 Hz processing loop
        

    def scan_callback(self, scan_msg):
        self.scan_msg = scan_msg
    
    def processing(self, event):
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
        if is_u_shape:
            print("U-shape parking structure detected")
        else:
            print("No parking")
        
        # 이미 goal 보냈으면 또 안 보냄 (원하면 이 로직은 빼도 됨)
        if self.goal_published:
            return

        # lines로부터 goal pose 계산
        success, gx, gy, gyaw = compute_goal_from_lines(
            lines,
            min_width=1.8,   # 필요하면 파라미터화
            min_depth=3.5,
            wall_offset=0.5
        )

        if not success:
            print("U-shape is detected but not enough width/depth")
            return

        # goal pose publish
        publish_parking_goal(
            self.goal_pub,
            gx,
            gy,
            gyaw,
            frame_id="base_link"  # 지금은 lidar = base_link 기준
        )

        rospy.loginfo(
            "Parking goal published: x=%.2f, y=%.2f, yaw=%.1f deg",
            gx, gy, degrees(gyaw)
        )

        self.goal_published = True

def main():
    try:
        parking_mission = Parking_mission()
        rospy.spin()
    except rospy.ROSInternalException:
        pass

if __name__ == '__main__':
    main()    
