import math
from typing import List, Tuple, Dict
import numpy as np

def detect(angle_ranges, dist_ranges, eps, min_pts):

    if angle_ranges.size == 0 or dist_ranges.size == 0:
        return False, []
    points_xy = polar_to_cartesian(angle_ranges, dist_ranges)
    # print(points_xy)
    labels = dbscan(min_pts, eps, points_xy)
    centroids = compute_centroids(points_xy, labels)
    return bool(centroids), centroids

def polar_to_cartesian(angles: np.ndarray, dists: np.ndarray) -> np.ndarray:
    # (dist, radian) -> (x,y)
    x = dists * np.cos(angles)
    y = dists * np.sin(angles)
    return np.stack((x, y), axis=1)

def dbscan(min_pts, eps, points: np.ndarray) -> np.ndarray:

    if len(points) == 0:
        return np.array([], dtype=int)
    UNCLASSIFIED = -1 # all the points unclassified
    NOISE = -2 # noise threshold 
    labels = np.full(len(points), UNCLASSIFIED, dtype=int)
    cluster_id = 0
    for idx in range(len(points)):
        if labels[idx] != UNCLASSIFIED:
            continue
        neighbors = region_query(eps, points, idx)
        if len(neighbors) < min_pts:
            labels[idx] = NOISE
            continue
        labels[idx] = cluster_id
        seeds = set(neighbors)
        seeds.discard(idx)
        while seeds:
            current = seeds.pop()
            if labels[current] == NOISE:
                labels[current] = cluster_id
            if labels[current] != UNCLASSIFIED:
                continue
            labels[current] = cluster_id
            current_neighbors = region_query(eps, points, current)
            if len(current_neighbors) >= min_pts:
                seeds.update(current_neighbors)
        cluster_id += 1
    return labels

def region_query(eps, points: np.ndarray, index: int) -> List[int]:
    deltas = points - points[index]
    dists = np.hypot(deltas[:, 0], deltas[:, 1]) # np.hypot -> about (x,y), get Euclidean dist sqrt(x**2+y**2)
    return np.where(dists <= eps)[0].tolist() # get neighborhood in the eps 
    
def compute_centroids(points: np.ndarray, labels: np.ndarray) -> List[Dict[str, float]]:
    centroids: List[Dict[str, float]] = []
    valid_clusters = {label for label in labels if label >= 0}
    for cid in sorted(valid_clusters):
        cluster_points = points[labels == cid]
        if cluster_points.size == 0:
            continue
        mean_x, mean_y = cluster_points.mean(axis=0)
        distance = float(math.hypot(mean_x, mean_y))
        angle_rad = math.atan2(mean_y, mean_x)
        angle_deg = math.degrees(angle_rad)
        centroids.append({"angle": angle_deg, "distance": distance})
    return centroids
