#include "parking_lot_common.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace parking_lot
{
bool preprocessLidar(const sensor_msgs::LaserScan& scan,
                     std::vector<double>& angles_deg_out,
                     std::vector<double>& dists_out)
{
  const std::size_t n = scan.ranges.size();
  if (n == 0) return false;

  const int half_window = 3;
  const int mvg_window  = 2 * half_window + 1;  // 7
  const double range_max_replace = 10.0;        // 무한대/NaN 대체값

  std::vector<double> padded(n + 2 * half_window,
                             std::numeric_limits<double>::quiet_NaN());
  for (std::size_t i = 0; i < n; ++i)
  {
    padded[half_window + i] = scan.ranges[i];
  }

  std::vector<double> ranges(n, scan.range_max);
  for (std::size_t i = 0; i < n; ++i)
  {
    double sum = 0.0;
    for (int k = 0; k < mvg_window; ++k)
    {
      double v = padded[i + k];
      if (!std::isfinite(v)) v = scan.range_max;
      if (v > scan.range_max) v = range_max_replace;
      sum += v;
    }
    ranges[i] = sum / static_cast<double>(mvg_window);
    if (!std::isfinite(ranges[i]) || ranges[i] > scan.range_max)
    {
      ranges[i] = range_max_replace;
    }
  }

  std::vector<double> raw_angles(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    raw_angles[i] = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    if (raw_angles[i] < 0)
    {
      raw_angles[i] = std::fmod(raw_angles[i], 2 * M_PI) + 2 * M_PI;
    }
    else if (raw_angles[i] >= 2 * M_PI)
    {
      raw_angles[i] = std::fmod(raw_angles[i], 2 * M_PI);
    }
  }

  const double lower = M_PI;        // 180deg
  const double upper = 2.0 * M_PI;  // 360deg

  std::vector<double> sel_angles;
  std::vector<double> sel_ranges;
  sel_angles.reserve(n);
  sel_ranges.reserve(n);

  for (std::size_t i = 0; i < n; ++i)
  {
    const double a = raw_angles[i];
    if (a >= lower && a <= upper)
    {
      sel_angles.push_back(a);
      sel_ranges.push_back(ranges[i]);
    }
  }

  if (sel_angles.empty()) return false;

  std::vector<std::size_t> idx(sel_angles.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::sort(idx.begin(), idx.end(),
            [&](std::size_t i, std::size_t j) { return sel_angles[i] < sel_angles[j]; });

  std::vector<double> angles_sorted, ranges_sorted;
  angles_sorted.reserve(idx.size());
  ranges_sorted.reserve(idx.size());

  for (std::size_t id : idx)
  {
    angles_sorted.push_back(sel_angles[id]);
    ranges_sorted.push_back(sel_ranges[id]);
  }

  angles_deg_out.clear();
  dists_out.clear();
  for (std::size_t i = 0; i < angles_sorted.size(); ++i)
  {
    if (ranges_sorted[i] < 2.5)
    {
      angles_deg_out.push_back(angles_sorted[i] * 180.0 / M_PI);
      dists_out.push_back(ranges_sorted[i]);
    }
  }

  return !angles_deg_out.empty();
}
}  // namespace parking_lot
