#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "object_detect.hpp"

inline ros::Publisher initClusterVisualizer(ros::NodeHandle &nh, const std::string &topic = "rotary/obstacle_markers") {
  return nh.advertise<visualization_msgs::MarkerArray>(topic, 1);
}

inline void publishClusterMarkers(const ros::Publisher &publisher, const std::vector<ClusterCentroid> &centroids) {
  visualization_msgs::MarkerArray marker_array;
  const ros::Time timestamp = ros::Time::now();

  for (std::size_t cluster_id = 0; cluster_id < centroids.size(); ++cluster_id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "laser";
    marker.header.stamp = timestamp;
    marker.ns = "rotary_objects";
    marker.id = static_cast<int>(cluster_id);
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    const double angle_rad = centroids[cluster_id].angle * M_PI / 180.0;
    const double distance = centroids[cluster_id].distance;
    marker.pose.position.x = distance * std::cos(angle_rad);
    marker.pose.position.y = distance * std::sin(angle_rad);
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 0.2f;
    marker.color.g = 0.6f;
    marker.color.b = 1.0f;
    marker.color.a = 0.8f;

    marker_array.markers.push_back(marker);
  }

  publisher.publish(marker_array);
}
