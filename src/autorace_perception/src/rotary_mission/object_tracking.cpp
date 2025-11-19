#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "clustering_visualization.hpp"
#include "lidar_preprocessing.hpp"
#include "obstacle_detect.hpp"

class RotaryMission {
public:
  RotaryMission()
      : nh_(), visualizer_(), scan_sub_(nh_.subscribe("/scan", 1, &RotaryMission::scanCallback, this)),
        timer_(nh_.createTimer(ros::Duration(0.1), &RotaryMission::processing, this)) {}

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) { scan_msg_ = scan_msg; }

  void processing(const ros::TimerEvent &) {
    if (!scan_msg_) {
      return;
    }

    const LidarProcessingResult processed = preprocessLidar(*scan_msg_);
    const double eps = 0.3;
    const int min_pts = 7;
    const DetectionResult detection = detect(processed.angle_ranges_deg, processed.dist_ranges, eps, min_pts);

    if (detection.detected) {
      for (std::size_t cluster_id = 0; cluster_id < detection.centroids.size(); ++cluster_id) {
        const ClusterCentroid &center = detection.centroids[cluster_id];
        ROS_INFO("id=%zu angle(deg)=%.3f distance(m)=%.3f", cluster_id, center.angle, center.distance);
      }
      visualizer_.publish(detection.centroids);
    } else {
      ROS_INFO("not detected");
    }
  }

  ros::NodeHandle nh_;
  ClusterVisualizer visualizer_;
  ros::Subscriber scan_sub_;
  ros::Timer timer_;
  sensor_msgs::LaserScan::ConstPtr scan_msg_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotary_mission");
  try {
    RotaryMission mission;
    ros::spin();
  } catch (const ros::Exception &ex) {
    ROS_ERROR("Rotary mission failed: %s", ex.what());
  }
  return 0;
}
