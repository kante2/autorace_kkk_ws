#pragma once

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include "gate_types.hpp"

GateLidarPreprocessResult preprocessGateLidar(const sensor_msgs::LaserScan& scan,
                                              double range_min,
                                              double range_max,
                                              double front_min_deg,
                                              double front_max_deg,
                                              laser_geometry::LaserProjection& projector,
                                              bool make_cloud);
