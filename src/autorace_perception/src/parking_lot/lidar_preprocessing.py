import numpy as np
from math import acos, cos, pi, sqrt    

def preprocess_lidar(scan_msg):
    ranges_raw = np.array(scan_msg.ranges) # lidar scan data to array 
    
    half_window = 3 # the size of mean-movement filter
    mvg_window = 2 * half_window + 1 # for noise sampling by side
    padding = np.array([np.nan] * half_window) # edge processing
    
    ranges = np.append(np.append(padding, ranges_raw), padding) # distance applied filter
    ranges = np.convolve(ranges, np.ones(mvg_window), "valid") / mvg_window # smoothing distance 
    ranges = np.nan_to_num(ranges, nan=scan_msg.range_max)  # replace NaNs before thresholding
    ranges[np.isinf(ranges) | (ranges > scan_msg.range_max)] = 10 # maximum dist -> 10m

    raw_angles = scan_msg.angle_min + np.arange(len(ranges_raw)) * scan_msg.angle_increment
    angle_ranges = np.mod(raw_angles, 2 * pi)  # normalize to [0, 2π)

    lower_bound = 180 / 180 * pi
    upper_bound = 360 / 180 * pi
    mask = (angle_ranges <= upper_bound) & (angle_ranges >= lower_bound) # 120 ~ 240 

    dist_ranges = ranges[mask]
    angle_ranges = angle_ranges[mask]

    sort_idx = np.argsort(angle_ranges) # upper alignment
    angle_ranges = angle_ranges[sort_idx]
    dist_ranges = dist_ranges[sort_idx]

    close_mask = dist_ranges < 2.5 # threshold = 3m
    angle_ranges = angle_ranges[close_mask]
    dist_ranges = dist_ranges[close_mask]

    angle_ranges = np.rad2deg(angle_ranges)

    return angle_ranges, dist_ranges
