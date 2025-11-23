#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <vector>

struct ClusterCentroid {
  double angle;    // degrees
  double distance; // meters
};

using PointXY = std::array<double, 2>;

struct DetectionResult {
  bool detected;
  std::vector<PointXY> points_xy;  // (x, y) point cloud after filtering
  std::vector<int> labels;         // DBSCAN labels for each point (-1: noise)
  std::vector<ClusterCentroid> centroids;
};

inline std::vector<PointXY> polarToCartesian(const std::vector<double> &angles_deg,
                                             const std::vector<double> &dists) {
  std::vector<PointXY> points;
  if (angles_deg.size() != dists.size()) {
    return points;
  }

  points.reserve(angles_deg.size());
  for (std::size_t i = 0; i < angles_deg.size(); ++i) {
    const double angle_rad = angles_deg[i] * M_PI / 180.0;
    const double cos_val = std::cos(angle_rad);
    const double sin_val = std::sin(angle_rad);
    points.push_back({dists[i] * cos_val, dists[i] * sin_val});
  }
  return points;
}

inline std::vector<int> regionQuery(double eps, const std::vector<PointXY> &points, std::size_t index) {
  std::vector<int> neighbors;
  if (index >= points.size()) {
    return neighbors;
  }

  for (std::size_t i = 0; i < points.size(); ++i) {
    const double dx = points[i][0] - points[index][0];
    const double dy = points[i][1] - points[index][1];
    if (std::hypot(dx, dy) <= eps) {
      neighbors.push_back(static_cast<int>(i));
    }
  }
  return neighbors;
}

inline std::vector<int> dbscan(int min_pts, double eps, const std::vector<PointXY> &points) {
  if (points.empty()) {
    return {};
  }

  constexpr int UNCLASSIFIED = -1;
  constexpr int NOISE = -2;

  std::vector<int> labels(points.size(), UNCLASSIFIED);
  int cluster_id = 0;

  for (std::size_t idx = 0; idx < points.size(); ++idx) {
    if (labels[idx] != UNCLASSIFIED) {
      continue;
    }

    const std::vector<int> neighbors = regionQuery(eps, points, idx);
    if (static_cast<int>(neighbors.size()) < min_pts) {
      labels[idx] = NOISE;
      continue;
    }

    labels[idx] = cluster_id;
    std::vector<int> seeds = neighbors;
    seeds.erase(std::remove(seeds.begin(), seeds.end(), static_cast<int>(idx)), seeds.end());

    while (!seeds.empty()) {
      const int current = seeds.back();
      seeds.pop_back();

      if (labels[current] == NOISE) {
        labels[current] = cluster_id;
      }
      if (labels[current] != UNCLASSIFIED) {
        continue;
      }

      labels[current] = cluster_id;
      std::vector<int> current_neighbors = regionQuery(eps, points, current);
      if (static_cast<int>(current_neighbors.size()) >= min_pts) {
        for (const int neighbor : current_neighbors) {
          if (std::find(seeds.begin(), seeds.end(), neighbor) == seeds.end()) {
            seeds.push_back(neighbor);
          }
        }
      }
    }

    ++cluster_id;
  }

  return labels;
}

inline std::vector<ClusterCentroid> computeCentroids(const std::vector<PointXY> &points,
                                                     const std::vector<int> &labels) {
  std::vector<ClusterCentroid> centroids;
  if (points.size() != labels.size()) {
    return centroids;
  }

  std::vector<int> unique_labels;
  for (int label : labels) {
    if (label >= 0 &&
        std::find(unique_labels.begin(), unique_labels.end(), label) == unique_labels.end()) {
      unique_labels.push_back(label);
    }
  }

  std::sort(unique_labels.begin(), unique_labels.end());

  for (int cluster_label : unique_labels) {
    std::vector<PointXY> cluster_points;
    for (std::size_t i = 0; i < labels.size(); ++i) {
      if (labels[i] == cluster_label) {
        cluster_points.push_back(points[i]);
      }
    }

    if (cluster_points.empty()) {
      continue;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    for (const auto &pt : cluster_points) {
      sum_x += pt[0];
      sum_y += pt[1];
    }

    const double mean_x = sum_x / static_cast<double>(cluster_points.size());
    const double mean_y = sum_y / static_cast<double>(cluster_points.size());
    const double distance = std::hypot(mean_x, mean_y);
    const double angle_deg = std::atan2(mean_y, mean_x) * 180.0 / M_PI;
    centroids.push_back({angle_deg, distance});
  }

  return centroids;
}

inline DetectionResult detect(const std::vector<double> &angle_ranges_deg,
                              const std::vector<double> &dist_ranges, double eps, int min_pts) {
  DetectionResult result;
  if (angle_ranges_deg.empty() || dist_ranges.empty()) {
    result.detected = false;
    return result;
  }

  result.points_xy = polarToCartesian(angle_ranges_deg, dist_ranges);
  result.labels = dbscan(min_pts, eps, result.points_xy);
  result.centroids = computeCentroids(result.points_xy, result.labels);
  result.detected = !result.centroids.empty();
  return result;
}
