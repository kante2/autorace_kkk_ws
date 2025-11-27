#include "parking_lot_common.hpp"

#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace parking_lot
{
namespace
{
double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

inline int signWithTol(double v, double tol = 1e-6)
{
  if (v > tol) return 1;
  if (v < -tol) return -1;
  return 0;
}

bool lookupBaseHeading(tf2_ros::Buffer& buffer,
                       const std::string& target_frame,
                       const std::string& source_frame,
                       double& yaw_out)
{
  try
  {
    geometry_msgs::TransformStamped tf =
        buffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(0.2));
    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw_out = yaw;
    return true;
  }
  catch (const tf2::TransformException& e)
  {
    ROS_WARN("base_link heading lookup failed (%s->%s): %s",
             target_frame.c_str(), source_frame.c_str(), e.what());
    return false;
  }
}
}  // namespace

GoalResult computeGoalFromLines(const std::vector<LineInfo>& lines,
                                double min_width,
                                double min_depth,
                                double wall_offset,
                                double max_width,
                                double max_depth)
{
  GoalResult res;
  if (lines.size() < 3) return res;

  std::vector<LineInfo> top = lines;
  std::sort(top.begin(), top.end(),
            [](const LineInfo& a, const LineInfo& b) { return a.num_inliers > b.num_inliers; });
  top.resize(3);

  std::vector<std::pair<double, double>> normals;
  normals.reserve(3);
  for (const auto& ln : top)
  {
    normals.emplace_back(ln.a, ln.b);
  }

  int best_i = -1, best_j = -1;
  double best_dot = -1.0;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = i + 1; j < 3; ++j)
    {
      const double dot = std::abs(normals[i].first * normals[j].first +
                                  normals[i].second * normals[j].second);
      if (dot > best_dot)
      {
        best_dot = dot;
        best_i = i;
        best_j = j;
      }
    }
  }
  if (best_i < 0) return res;

  int k = 3 - best_i - best_j;

  LineInfo side1 = top[best_i];
  LineInfo side2 = top[best_j];
  LineInfo back  = top[k];

  if (side1.a * side2.a + side1.b * side2.b < 0)
  {
    side2.a = -side2.a;
    side2.b = -side2.b;
    side2.c = -side2.c;
  }

  const double width = std::abs(side2.c - side1.c);
  if (width < min_width) return res;
  if (max_width > 0.0 && width > max_width) return res;

  const double depth = std::abs(back.c);
  if (depth < min_depth) return res;
  if (max_depth > 0.0 && depth > max_depth) return res;

  double n_back_x = back.a;
  double n_back_y = back.b;
  double cx = back.centroid_x;
  double cy = back.centroid_y;

  double vx = -cx;
  double vy = -cy;
  if (n_back_x * vx + n_back_y * vy < 0)
  {
    n_back_x = -n_back_x;
    n_back_y = -n_back_y;
  }

  double goal_x = cx + n_back_x * wall_offset;
  double goal_y = cy + n_back_y * wall_offset;

  // -------------------- 내부 포함성 체크 --------------------
  // back 라인의 센트로이드를 내부 기준점으로 사용해 각 선의 내부 방향을 결정
  const int interior_side1 = signWithTol(side1.a * cx + side1.b * cy + side1.c);
  const int interior_side2 = signWithTol(side2.a * cx + side2.b * cy + side2.c);
  const int interior_back  = signWithTol(n_back_x * cx + n_back_y * cy);  // back 기준점에서 내부는 back 노멀과 같은 방향

  auto isInsideLine = [](const LineInfo& ln, double x, double y, int interior_sign)
  {
    if (interior_sign == 0) return true;  // 불명확하면 패스
    const double s = ln.a * x + ln.b * y + ln.c;
    return signWithTol(s) == interior_sign;
  };

  // goal이 두 측면과 back 라인 내부에 있는지 확인
  if (!isInsideLine(side1, goal_x, goal_y, interior_side1) ||
      !isInsideLine(side2, goal_x, goal_y, interior_side2) ||
      signWithTol(n_back_x * goal_x + n_back_y * goal_y) != interior_back)
  {
    return res;  // 내부가 아니면 실패 처리
  }

  // 평행 두 선 사이에 goal이 실제로 위치하는지 추가 확인 (같은 노멀 방향에서 반대 부호여야 사이에 존재)
  const double s1_goal = side1.a * goal_x + side1.b * goal_y + side1.c;
  const double s2_goal = side2.a * goal_x + side2.b * goal_y + side2.c;
  if (signWithTol(s1_goal) != 0 && signWithTol(s2_goal) != 0 &&
      signWithTol(s1_goal) == signWithTol(s2_goal))
  {
    return res;  // 두 평행선 바깥에 goal이 위치 -> 실패
  }

  res.x = goal_x;
  res.y = goal_y;
  res.yaw = normalizeAngle(std::atan2(n_back_y, n_back_x) - M_PI / 2.0);
  res.success = true;
  return res;
}

bool publishParkingGoal(ros::Publisher& pub,
                        tf2_ros::Buffer& tf_buffer,
                        double goal_x,
                        double goal_y,
                        double goal_yaw,
                        const std::string& frame_id,
                        const std::string& target_frame,
                        bool align_heading,
                        geometry_msgs::PoseStamped& out_msg)
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time(0);
  msg.header.frame_id = frame_id;
  msg.pose.position.x = goal_x;
  msg.pose.position.y = goal_y;
  msg.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, goal_yaw);
  msg.pose.orientation = tf2::toMsg(q);

  geometry_msgs::PoseStamped transformed = msg;

  if (!target_frame.empty() && target_frame != frame_id)
  {
    try
    {
      transformed = tf_buffer.transform(msg, target_frame, ros::Duration(0.2));
    }
    catch (const tf2::TransformException& e)
    {
      ROS_WARN("goal transform failed (%s->%s): %s", frame_id.c_str(),
               target_frame.c_str(), e.what());
      return false;
    }

    if (align_heading)
    {
      double base_yaw = 0.0;
      if (lookupBaseHeading(tf_buffer, target_frame, frame_id, base_yaw))
      {
        tf2::Quaternion qh;
        qh.setRPY(0.0, 0.0, base_yaw);
        transformed.pose.orientation = tf2::toMsg(qh);
      }
    }
  }

  pub.publish(transformed);
  out_msg = transformed;
  return true;
}
}  // namespace parking_lot
