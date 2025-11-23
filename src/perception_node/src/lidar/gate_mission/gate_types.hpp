#pragma once

#include <limits>
#include <vector>

#include <sensor_msgs/PointCloud2.h>

// 2D 포인트
struct Point2D
{
  double x;
  double y;
};

// 게이트 상태
enum GateState
{
  GATE_UP = 0,
  GATE_DOWN = 1
};

// 라이다 전처리 결과
struct GateLidarPreprocessResult
{
  std::vector<Point2D> points;
  sensor_msgs::PointCloud2 cloud;
  bool has_cloud = false;
};

// 게이트 감지 결과
struct GateDetectionResult
{
  bool has_gate = false;
  double gate_cx = 0.0;
  double gate_cy = 0.0;
  double gate_dist = std::numeric_limits<double>::infinity();
  GateState gate_state = GATE_UP;
  int cluster_count = 0;
  bool gate_down_final = false;
  std::vector<Point2D> gate_cluster;
};
