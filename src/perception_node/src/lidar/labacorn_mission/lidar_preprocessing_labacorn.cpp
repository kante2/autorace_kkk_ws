// lidar_preprocessing_labacorn.cpp
// LaserScan -> std::vector<Point2D> 전처리 (원본 로직 보존)

#include "object_detect_laba.hpp" // Point2D
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <cmath>
#include <limits>

// 전역 파라미터는 mission_labacorn.cpp 에 정의되어 있고 extern으로 사용됨
extern double g_range_min;
extern double g_range_max;
extern double g_front_min_deg;
extern double g_front_max_deg;

// inline clamp (로컬 정의, inline이면 링크 충돌 없음)
inline double clamp_local(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

std::vector<Point2D> filterScanPoints(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  std::vector<Point2D> points;
  points.reserve(scan->ranges.size());

  double angle = scan->angle_min;

  for (auto r : scan->ranges)
  {
    // 거리 필터
    if (!std::isfinite(r) || r < g_range_min || r > g_range_max)
    {
      angle += scan->angle_increment;
      continue;
    }

    // 각도 필터 (degree 기준)
    double angle_deg = angle * 180.0 / M_PI;
    if (angle_deg < 0.0) angle_deg += 360.0;

    if (angle_deg < g_front_min_deg || angle_deg > g_front_max_deg)
    {
      angle += scan->angle_increment;
      continue;
    }

    Point2D p;
    p.x = r * std::cos(angle);
    p.y = r * std::sin(angle);
    points.push_back(p);

    angle += scan->angle_increment;
  }

  return points;
}
