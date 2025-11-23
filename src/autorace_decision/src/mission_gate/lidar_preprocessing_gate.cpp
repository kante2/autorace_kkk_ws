#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

#include <cmath>
#include <vector>

struct GateLidarPreprocessResult
{
  std::vector<Point2D> points;
  sensor_msgs::PointCloud2 cloud;
  bool has_cloud = false;
};

// LaserScan -> ROI 필터링된 포인트 목록(+옵션으로 PointCloud2 생성)
GateLidarPreprocessResult preprocessGateLidar(const sensor_msgs::LaserScan& scan,
                                              double range_min,
                                              double range_max,
                                              double front_min_deg,
                                              double front_max_deg,
                                              laser_geometry::LaserProjection& projector,
                                              bool make_cloud)
{
  GateLidarPreprocessResult result;
  result.points.reserve(scan.ranges.size());

  double angle = scan.angle_min;

  for (const auto& r : scan.ranges)
  {
    if (!std::isfinite(r) || r < range_min || r > range_max)
    {
      angle += scan.angle_increment;
      continue;
    }

    double angle_deg = angle * 180.0 / M_PI;
    if (angle_deg < 0.0)
      angle_deg += 360.0;

    if (angle_deg < front_min_deg || angle_deg > front_max_deg)
    {
      angle += scan.angle_increment;
      continue;
    }

    double x = r * std::cos(angle);
    double y = r * std::sin(angle);
    result.points.push_back({x, y});

    angle += scan.angle_increment;
  }

  if (make_cloud)
  {
    projector.projectLaser(scan, result.cloud);
    result.has_cloud = true;
  }

  return result;
}
