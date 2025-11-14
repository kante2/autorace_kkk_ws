import math
from typing import List, Dict

import rospy
from visualization_msgs.msg import Marker, MarkerArray


class ClusterVisualizer:
    """Publish DBSCAN centroids as RViz markers."""

    def __init__(self, topic: str = "rotary/obstacle_markers"):
        self.publisher = rospy.Publisher(topic, MarkerArray, queue_size=1)

    def publish(self, centroids: List[Dict[str, float]]):
        marker_array = MarkerArray()
        timestamp = rospy.Time.now()

        for cluster_id, center in enumerate(centroids):
            marker = Marker()
            marker.header.frame_id = "laser"
            marker.header.stamp = timestamp
            marker.ns = "dbscan_clusters"
            marker.id = cluster_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            angle_rad = math.radians(center.get("angle", 0.0))
            distance = center.get("distance", 0.0)
            marker.pose.position.x = distance * math.cos(angle_rad)
            marker.pose.position.y = distance * math.sin(angle_rad)
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.r = 0.2
            marker.color.g = 0.6
            marker.color.b = 1.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        self.publisher.publish(marker_array)
