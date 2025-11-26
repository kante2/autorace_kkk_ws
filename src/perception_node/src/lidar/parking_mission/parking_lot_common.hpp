#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/buffer.h>

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace parking_lot
{
struct LineInfo
{
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
  int num_inliers = 0;
  std::vector<int> indices;
  double centroid_x = 0.0;
  double centroid_y = 0.0;
};

struct GoalResult
{
  bool success = false;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// lidar_preprocessing.cpp
bool preprocessLidar(const sensor_msgs::LaserScan& scan,
                     std::vector<double>& angles_deg_out,
                     std::vector<double>& dists_out);

// parking_lot_detection.cpp
std::pair<bool, std::vector<LineInfo>> parkingDetect(const std::vector<double>& angles_deg,
                                                     const std::vector<double>& dists,
                                                     int ransac_max_lines,
                                                     int ransac_max_iters,
                                                     double ransac_dist_thresh,
                                                     int ransac_min_inliers,
                                                     int max_line_inliers,
                                                     double parallel_angle_deg,
                                                     double orth_angle_deg);

// goal_pose_pub.cpp
GoalResult computeGoalFromLines(const std::vector<LineInfo>& lines,
                                double min_width,
                                double min_depth,
                                double wall_offset,
                                double max_width = -1.0,
                                double max_depth = -1.0);

bool publishParkingGoal(ros::Publisher& pub,
                        tf2_ros::Buffer& tf_buffer,
                        double goal_x,
                        double goal_y,
                        double goal_yaw,
                        const std::string& frame_id,
                        const std::string& target_frame,
                        bool align_heading,
                        geometry_msgs::PoseStamped& out_msg);
}  // namespace parking_lot
