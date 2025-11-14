import rospy 
from math import *
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64
import numpy as np
from lidar_preprocessing import preprocess_lidar
from obstacle_detect import detect
from clustering_visualization import ClusterVisualizer
class Rotary_mission:
    def __init__(self):
        rospy.init_node("rotary_mission")
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        self.scan_msg = None
        self.preprocess_lidar = preprocess_lidar
        self.detect = detect
        self.visualizer = ClusterVisualizer()
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

        eps = 0.3
        min_pts = 7
        detected, centroids = self.detect(angle_ranges, dist_ranges, eps, min_pts)
        if detected:
            for cluster_id, center in enumerate(centroids):
                angle = center["angle"]
                distance = center["distance"]
                print(f"id={cluster_id} angle(deg)={angle:.3f} distance(m)={distance:.3f}")
            self.visualizer.publish(centroids)
        else:
            print("not detected")

def main():
    try:
        rotary_mission = Rotary_mission()
        rospy.spin()
    except rospy.ROSInternalException:
        pass

if __name__ == '__main__':
    main()    
