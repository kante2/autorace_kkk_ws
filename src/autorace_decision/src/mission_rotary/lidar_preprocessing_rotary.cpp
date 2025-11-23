#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <sensor_msgs/LaserScan.h>

struct LidarProcessingResult {
  std::vector<double> angle_ranges_deg;
  std::vector<double> dist_ranges;
};

inline double normalizeAnglePositive(double angle) {
  const double two_pi = 2.0 * M_PI;
  double norm = std::fmod(angle, two_pi);
  if (norm < 0.0) {
    norm += two_pi;
  }
  return norm;
}

inline LidarProcessingResult preprocessLidar(const sensor_msgs::LaserScan &scan_msg) {
  const std::vector<float> &ranges_raw = scan_msg.ranges;
  const std::size_t raw_size = ranges_raw.size();

  constexpr std::size_t half_window = 3;
  constexpr std::size_t mvg_window = 2 * half_window + 1;
  const double nan_value = std::numeric_limits<double>::quiet_NaN();

  std::vector<double> padded;
  padded.reserve(raw_size + 2 * half_window);
  padded.insert(padded.end(), half_window, nan_value);
  for (float value : ranges_raw) {
    padded.push_back(static_cast<double>(value));
  }
  padded.insert(padded.end(), half_window, nan_value);

  std::vector<double> filtered(raw_size, scan_msg.range_max);
  for (std::size_t i = 0; i < raw_size; ++i) {
    bool contains_nan = false;
    double sum = 0.0;
    for (std::size_t j = 0; j < mvg_window; ++j) {
      const double sample = padded[i + j];
      if (std::isnan(sample)) {
        contains_nan = true;
        break;
      }
      sum += sample;
    }
    filtered[i] = contains_nan ? nan_value : sum / static_cast<double>(mvg_window);
  }

  for (double &value : filtered) {
    if (std::isnan(value)) {
      value = scan_msg.range_max;
    }
    if (!std::isfinite(value) || value > static_cast<double>(scan_msg.range_max)) {
      value = 10.0;
    }
  }

  std::vector<double> angle_ranges;
  angle_ranges.reserve(raw_size);

  double angle = scan_msg.angle_min;
  for (std::size_t i = 0; i < raw_size; ++i) {
    angle_ranges.push_back(normalizeAnglePositive(angle));
    angle += scan_msg.angle_increment;
  }

  const double lower_bound = 120.0 / 180.0 * M_PI;
  const double upper_bound = 240.0 / 180.0 * M_PI;

  struct PolarSample {
    double angle;
    double distance;
  };

  std::vector<PolarSample> masked;
  for (std::size_t i = 0; i < angle_ranges.size(); ++i) {
    const double ang = angle_ranges[i];
    if (ang >= lower_bound && ang <= upper_bound) {
      masked.push_back({ang, filtered[i]});
    }
  }

  std::sort(masked.begin(), masked.end(),
            [](const PolarSample &lhs, const PolarSample &rhs) { return lhs.angle < rhs.angle; });

  LidarProcessingResult result;
  for (const auto &sample : masked) {
    if (sample.distance < 2.5) {
      result.angle_ranges_deg.push_back(sample.angle * 180.0 / M_PI);
      result.dist_ranges.push_back(sample.distance);
    }
  }

  return result;
}