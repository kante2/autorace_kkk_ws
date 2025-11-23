#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include "obstacle_detect.hpp"

/**
 * @brief Publish DBSCAN clusters (points + centroids) as RViz markers.
 */
class ClusterVisualizer {
public:
  explicit ClusterVisualizer(const std::string &topic = "rotary/obstacle_markers")
      : publisher_(nh_.advertise<visualization_msgs::MarkerArray>(topic, 1)) {}

  void publish(const DetectionResult &det) {
    visualization_msgs::MarkerArray marker_array;
    const ros::Time stamp = ros::Time::now();

    // --- Centroid markers ---
    for (std::size_t i = 0; i < det.centroids.size(); ++i) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "laser";
      marker.header.stamp = stamp;
      marker.ns = "dbscan_centroids";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;

      const double angle_rad = det.centroids[i].angle * M_PI / 180.0;
      const double distance = det.centroids[i].distance;
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
      marker.color.a = 0.9f;

      marker_array.markers.push_back(marker);
    }

    // --- Cluster point markers (one SPHERE_LIST per cluster) ---
    if (det.points_xy.size() == det.labels.size()) {
      // Find unique cluster labels (>=0)
      std::vector<int> labels = det.labels;
      std::sort(labels.begin(), labels.end());
      labels.erase(std::unique(labels.begin(), labels.end()), labels.end());

      int marker_id = 1000;  // separate namespace ids
      for (int label : labels) {
        if (label < 0) {
          continue;  // skip noise
        }

        visualization_msgs::Marker points_marker;
        points_marker.header.frame_id = "laser";
        points_marker.header.stamp = stamp;
        points_marker.ns = "dbscan_clusters";
        points_marker.id = marker_id++;
        points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        points_marker.action = visualization_msgs::Marker::ADD;
        points_marker.scale.x = 0.08;
        points_marker.scale.y = 0.08;
        points_marker.scale.z = 0.08;

        // Color from label (simple hash)
        std_msgs::ColorRGBA c;
        const double hue = (label * 70) % 360;  // vary hue
        const double rad = hue * M_PI / 180.0;
        c.r = static_cast<float>(0.6 + 0.4 * std::cos(rad));
        c.g = static_cast<float>(0.6 + 0.4 * std::cos(rad + 2.094));  // +120 deg
        c.b = static_cast<float>(0.6 + 0.4 * std::cos(rad + 4.188));  // +240 deg
        c.a = 0.9f;
        points_marker.color = c;

        for (std::size_t idx = 0; idx < det.labels.size(); ++idx) {
          if (det.labels[idx] != label) {
            continue;
          }
          geometry_msgs::Point p;
          p.x = det.points_xy[idx][0];
          p.y = det.points_xy[idx][1];
          p.z = 0.0;
          points_marker.points.push_back(p);
        }

        if (!points_marker.points.empty()) {
          marker_array.markers.push_back(points_marker);
        }
      }
    }

    publisher_.publish(marker_array);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
};
