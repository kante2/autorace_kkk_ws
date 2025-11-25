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

  void publish(const DetectionResult &det, const std::vector<bool> &dynamic_flags) {
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

      const bool is_dynamic = (i < dynamic_flags.size()) ? dynamic_flags[i] : false;
      if (is_dynamic) {
        marker.color.r = 0.1f;
        marker.color.g = 0.5f;
        marker.color.b = 1.0f;  // 파란색 계열 (동적)
      } else {
        marker.color.r = 1.0f;
        marker.color.g = 0.2f;
        marker.color.b = 0.2f;  // 빨간색 계열 (정적)
      }
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

        // 색상: 정적 빨간색, 동적 파란색
        const bool is_dynamic =
            (label >= 0 && static_cast<std::size_t>(label) < dynamic_flags.size())
                ? dynamic_flags[static_cast<std::size_t>(label)]
                : false;
        if (is_dynamic) {
          points_marker.color.r = 0.1f;
          points_marker.color.g = 0.5f;
          points_marker.color.b = 1.0f;
        } else {
          points_marker.color.r = 1.0f;
          points_marker.color.g = 0.2f;
          points_marker.color.b = 0.2f;
        }
        points_marker.color.a = 0.9f;

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
