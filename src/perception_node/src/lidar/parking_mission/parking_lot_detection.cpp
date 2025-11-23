#include "parking_lot_common.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

namespace parking_lot
{
namespace
{
bool lineFromPoints(const Eigen::Vector2d& p1,
                    const Eigen::Vector2d& p2,
                    double& a,
                    double& b,
                    double& c)
{
  const double dx = p2.x() - p1.x();
  const double dy = p2.y() - p1.y();
  if (std::hypot(dx, dy) < 1e-6) return false;

  a = dy;
  b = -dx;
  const double norm = std::hypot(a, b);
  if (norm < 1e-9) return false;
  a /= norm;
  b /= norm;
  c = -(a * p1.x() + b * p1.y());
  return true;
}

std::vector<LineInfo> extractLinesRansac(const std::vector<Eigen::Vector2d>& points,
                                         int max_lines,
                                         int max_iters,
                                         double dist_thresh,
                                         int min_inliers)
{
  std::vector<LineInfo> lines;
  if (points.empty()) return lines;

  std::vector<Eigen::Vector2d> remaining = points;
  std::mt19937 rng(std::random_device{}());

  for (int l = 0; l < max_lines; ++l)
  {
    const int N = static_cast<int>(remaining.size());
    if (N < min_inliers) break;

    LineInfo best;
    bool found = false;

    std::uniform_int_distribution<int> dist_idx(0, N - 1);

    for (int it = 0; it < max_iters; ++it)
    {
      int i1 = dist_idx(rng);
      int i2 = dist_idx(rng);
      if (i1 == i2) continue;

      double a, b, c;
      if (!lineFromPoints(remaining[i1], remaining[i2], a, b, c)) continue;

      std::vector<int> inliers;
      inliers.reserve(N);
      for (int i = 0; i < N; ++i)
      {
        const double d = std::abs(a * remaining[i].x() + b * remaining[i].y() + c);
        if (d < dist_thresh) inliers.push_back(i);
      }

      if (static_cast<int>(inliers.size()) > best.num_inliers)
      {
        found = true;
        best.num_inliers = static_cast<int>(inliers.size());
        best.a = a;
        best.b = b;
        best.c = c;
        best.indices = inliers;

        Eigen::Vector2d sum = Eigen::Vector2d::Zero();
        for (int idx : inliers) sum += remaining[idx];
        sum /= static_cast<double>(inliers.size());
        best.centroid_x = sum.x();
        best.centroid_y = sum.y();
      }
    }

    if (!found || best.num_inliers < min_inliers) break;
    lines.push_back(best);

    std::vector<Eigen::Vector2d> next;
    next.reserve(remaining.size() - best.num_inliers);
    std::vector<bool> is_inlier(remaining.size(), false);
    for (int idx : best.indices) is_inlier[idx] = true;
    for (std::size_t i = 0; i < remaining.size(); ++i)
    {
      if (!is_inlier[i]) next.push_back(remaining[i]);
    }
    remaining.swap(next);
  }

  return lines;
}

bool checkUShape(std::vector<LineInfo> lines,
                 double parallel_angle_deg,
                 double orth_angle_deg)
{
  if (lines.size() < 3) return false;

  std::sort(lines.begin(), lines.end(),
            [](const LineInfo& a, const LineInfo& b) { return a.num_inliers > b.num_inliers; });
  lines.resize(3);

  struct NormalInfo
  {
    Eigen::Vector2d n;
    double theta;
  };
  std::vector<NormalInfo> normals;
  normals.reserve(3);
  for (const auto& ln : lines)
  {
    Eigen::Vector2d n(ln.a, ln.b);
    normals.push_back({n, std::atan2(n.y(), n.x())});
  }

  int best_i = -1, best_j = -1;
  double best_dot = -1.0;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = i + 1; j < 3; ++j)
    {
      const double dot = std::abs(normals[i].n.dot(normals[j].n));
      if (dot > best_dot)
      {
        best_dot = dot;
        best_i = i;
        best_j = j;
      }
    }
  }
  if (best_i < 0) return false;

  const int k = 3 - best_i - best_j;

  const double delta_parallel = std::acos(std::clamp(best_dot, -1.0, 1.0));
  if (delta_parallel > parallel_angle_deg * M_PI / 180.0) return false;

  const auto& nk = normals[k].n;
  const auto& ni = normals[best_i].n;
  const auto& nj = normals[best_j].n;

  const double angle_ki = std::acos(std::clamp(std::abs(nk.dot(ni)), -1.0, 1.0));
  const double angle_kj = std::acos(std::clamp(std::abs(nk.dot(nj)), -1.0, 1.0));
  const double dev_ki = std::abs(angle_ki - M_PI / 2.0);
  const double dev_kj = std::abs(angle_kj - M_PI / 2.0);

  const double orth_tol = orth_angle_deg * M_PI / 180.0;
  if (dev_ki > orth_tol || dev_kj > orth_tol) return false;

  return true;
}
}  // namespace

std::pair<bool, std::vector<LineInfo>> parkingDetect(const std::vector<double>& angles_deg,
                                                     const std::vector<double>& dists,
                                                     int ransac_max_lines,
                                                     int ransac_max_iters,
                                                     double ransac_dist_thresh,
                                                     int ransac_min_inliers,
                                                     double parallel_angle_deg,
                                                     double orth_angle_deg)
{
  if (angles_deg.size() != dists.size() || angles_deg.empty())
    return {false, {}};

  std::vector<Eigen::Vector2d> points;
  points.reserve(angles_deg.size());
  for (std::size_t i = 0; i < angles_deg.size(); ++i)
  {
    const double angle_rad = angles_deg[i] * M_PI / 180.0 - M_PI;  // 전방을 +x로 맞춤
    const double r = dists[i];
    points.emplace_back(r * std::cos(angle_rad), r * std::sin(angle_rad));
  }

  if (points.size() < static_cast<std::size_t>(ransac_min_inliers * 3))
    return {false, {}};

  auto lines = extractLinesRansac(points,
                                  ransac_max_lines,
                                  ransac_max_iters,
                                  ransac_dist_thresh,
                                  ransac_min_inliers);

  const bool is_u = checkUShape(lines, parallel_angle_deg, orth_angle_deg);
  return {is_u, lines};
}
}  // namespace parking_lot
