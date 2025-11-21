import math
from typing import List, Tuple, Dict
import numpy as np
import random
from math import radians

def parking_detect(angle_ranges, dist_ranges,
                   ransac_max_lines=5,
                   ransac_max_iters=200,
                   ransac_dist_thresh=0.05,
                   ransac_min_inliers=30,
                   parallel_angle_deg=15.0,
                   orth_angle_deg=15.0):
    """
    angle_ranges_deg, dist_ranges를 입력으로 ㄷ자 구조 존재 여부를 반환
    """
    # 1. 폴라 → 직교 좌표
    points = polar_to_cartesian(angle_ranges, dist_ranges)

    # 포인트가 너무 적으면 바로 False
    if points.shape[0] < ransac_min_inliers * 3:
        return False, []

    # 2. RANSAC으로 직선 세그먼트 추출
    lines = extract_lines_ransac(points,
                                 max_lines=ransac_max_lines,
                                 max_iters=ransac_max_iters,
                                 dist_thresh=ransac_dist_thresh,
                                 min_inliers=ransac_min_inliers)

    # 3. ㄷ자 구조인지 판별
    is_u = check_u_shape(lines,
                         parallel_angle_deg=parallel_angle_deg,
                         orth_angle_deg=orth_angle_deg)

    return is_u, lines

def polar_to_cartesian(angles, dists):
    """
    (dist, deg) -> (x,y)
    라이다 각도 기준: 후방=0/360°, 전방=180°, 반시계 증가.
    base_link 전방(+x)을 0°로 맞추기 위해 각도를 -pi 회전하여 좌표 변환.
    """
    angles = np.deg2rad(angles) - np.pi  # 180deg -> 0deg (전방 정렬)
    x = dists * np.cos(angles)
    y = dists * np.sin(angles)
    return np.vstack((x, y)).T

def line_from_points(p1, p2):
    """
    두 점 p1, p2로부터 정규화된 직선 ax + by + c = 0 계산
    a^2 + b^2 = 1
    """
    x1, y1 = p1
    x2, y2 = p2

    dx = x2 - x1
    dy = y2 - y1

    # 두 점이 거의 겹치면 무효
    if np.hypot(dx, dy) < 1e-6:
        return None

    # 직선 법선 벡터 (a, b) = (dy, -dx)
    a = dy
    b = -dx
    norm = np.hypot(a, b)
    a /= norm
    b /= norm

    # p1이 직선 위: a x1 + b y1 + c = 0 → c = -(a x1 + b y1)
    c = -(a * x1 + b * y1)
    return a, b, c

def extract_lines_ransac(points,
                         max_lines=5,
                         max_iters=200,
                         dist_thresh=0.05,
                         min_inliers=30):
    """
    RANSAC으로 여러 직선 세그먼트 추출
    points: (N, 2) numpy array
    반환: 리스트 [ {a,b,c,num_inliers,centroid,indices}, ... ]
    """
    remaining = points.copy()
    all_lines = []

    for _ in range(max_lines):
        N = remaining.shape[0]
        if N < min_inliers:
            break

        best_line = None
        best_inliers_idx = None
        best_inlier_count = 0

        for _ in range(max_iters):
            # 서로 다른 두 점 랜덤 샘플
            i1, i2 = random.sample(range(N), 2)
            p1 = remaining[i1]
            p2 = remaining[i2]

            params = line_from_points(p1, p2)
            if params is None:
                continue
            a, b, c = params

            # 모든 점에 대해 거리 계산
            dists = np.abs(a * remaining[:, 0] + b * remaining[:, 1] + c)

            inliers_idx = np.where(dists < dist_thresh)[0]
            inlier_count = inliers_idx.size

            if inlier_count > best_inlier_count:
                best_inlier_count = inlier_count
                best_inliers_idx = inliers_idx
                best_line = (a, b, c)

        if best_line is None or best_inlier_count < min_inliers:
            break

        a, b, c = best_line
        inlier_points = remaining[best_inliers_idx]
        centroid = np.mean(inlier_points, axis=0)

        line_info = {
            "a": a,
            "b": b,
            "c": c,
            "num_inliers": best_inlier_count,
            "centroid": centroid,
            "indices": best_inliers_idx
        }
        all_lines.append(line_info)

        # inlier 제거하고 다음 직선 찾기
        mask = np.ones(remaining.shape[0], dtype=bool)
        mask[best_inliers_idx] = False
        remaining = remaining[mask]

    return all_lines

def check_u_shape(lines,
                  parallel_angle_deg=15.0,
                  orth_angle_deg=15.0):
    """
    3개의 직선 정보가 ㄷ(U)자 구조인지 판별
    lines: 길이 >= 3 리스트, 각 원소는 {a,b,c,num_inliers,centroid,...}
    parallel_angle_deg: 평행 허용 각도
    orth_angle_deg: 직교 허용 각도
    """
    if len(lines) < 3:
        return False

    # 상위 3개만 사용
    lines = sorted(lines, key=lambda x: -x["num_inliers"])[:3]

    normals = []
    for ln in lines:
        a = ln["a"]
        b = ln["b"]
        n = np.array([a, b])  # 이미 정규화됨
        theta = np.arctan2(b, a)  # 법선 방향
        normals.append({
            "n": n,
            "theta": theta
        })

    # 3개 중 평행한 두 라인 찾기
    best_pair = None
    best_dot = -1.0

    for i in range(3):
        for j in range(i + 1, 3):
            ni = normals[i]["n"]
            nj = normals[j]["n"]
            dot = np.abs(np.dot(ni, nj))  # 방향만 보면 되므로 절대값
            if dot > best_dot:
                best_dot = dot
                best_pair = (i, j)

    if best_pair is None:
        return False

    i, j = best_pair
    # 나머지 하나 인덱스
    k = [0, 1, 2]
    k.remove(i)
    k.remove(j)
    k = k[0]

    # 평행 각도 체크
    # dot = cos(delta_theta)
    delta_parallel = np.arccos(np.clip(best_dot, -1.0, 1.0))
    if delta_parallel > radians(parallel_angle_deg):
        return False

    # 직교 체크
    nk = normals[k]["n"]
    ni = normals[i]["n"]
    nj = normals[j]["n"]

    # 각도 = arccos(|dot|)
    angle_ki = np.arccos(np.clip(np.abs(np.dot(nk, ni)), -1.0, 1.0))
    angle_kj = np.arccos(np.clip(np.abs(np.dot(nk, nj)), -1.0, 1.0))

    dev_ki = np.abs(angle_ki - np.pi / 2.0)
    dev_kj = np.abs(angle_kj - np.pi / 2.0)

    if dev_ki > radians(orth_angle_deg) or dev_kj > radians(orth_angle_deg):
        return False

    return True
