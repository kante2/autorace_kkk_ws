#!/usr/bin/env python3
import sys
import cv2
import numpy as np

# -------------------- 파라미터 --------------------
g_lane_width_px = 340.0

g_num_windows      = 12
g_window_margin    = 80
g_minpix_recenter  = 50
g_min_lane_sep     = 60
g_center_ema_alpha = 0.8

g_roi_top_y_ratio     = 0.60
g_roi_left_top_ratio  = 0.22
g_roi_right_top_ratio = 0.78
g_roi_left_bot_ratio  = -0.40
g_roi_right_bot_ratio = 1.40

g_yellow_lower = (15, 100, 80)   # H, S, V
g_yellow_upper = (40, 255, 255)  # H, S, V


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def make_roi_polygon(h, w):
    y_top = int(h * g_roi_top_y_ratio)
    y_bot = h - 1
    x_lt  = int(w * g_roi_left_top_ratio)
    x_rt  = int(w * g_roi_right_top_ratio)
    x_lb  = int(w * g_roi_left_bot_ratio)
    x_rb  = int(w * g_roi_right_bot_ratio)
    poly = np.array([[x_lb, y_bot], [x_lt, y_top], [x_rt, y_top], [x_rb, y_bot]], dtype=np.float32)
    poly[:, 1] = np.clip(poly[:, 1], 0, h - 1)
    return poly


def warp_to_bev(bgr, roi_poly):
    h, w = bgr.shape[:2]
    dst = np.array([[0, h - 1], [0, 0], [w - 1, 0], [w - 1, h - 1]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(roi_poly, dst)
    bev = cv2.warpPerspective(bgr, M, (w, h), flags=cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
    return bev


def binarize_lanes(bgr):
    bgr_blur = cv2.GaussianBlur(bgr, (5, 5), 0)
    hsv = cv2.cvtColor(bgr_blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, g_yellow_lower, g_yellow_upper)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    return mask


def run_sliding_window(binary_mask):
    h, w = binary_mask.shape[:2]
    debug_img = cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2BGR)

    lower = binary_mask[h // 2:, :]
    hist = np.sum(lower, axis=0)
    midpoint = w // 2
    left_base = np.argmax(hist[:midpoint]) if hist[:midpoint].size else -1
    right_base = np.argmax(hist[midpoint:]) + midpoint if hist[midpoint:].size else -1

    window_height = h // g_num_windows
    left_current, right_current = left_base, right_base

    nz_pts = cv2.findNonZero(binary_mask)
    nz_pts = nz_pts[:, 0, :] if nz_pts is not None else np.empty((0, 2), dtype=np.int32)

    left_centers, right_centers = [], []

    for win in range(g_num_windows):
        win_y_low = h - (win + 1) * window_height
        win_y_high = h - win * window_height

        win_x_left_low  = left_current  - g_window_margin
        win_x_left_high = left_current  + g_window_margin
        win_x_right_low  = right_current - g_window_margin
        win_x_right_high = right_current + g_window_margin

        if left_current >= 0:
            cv2.rectangle(debug_img, (win_x_left_low, win_y_low), (win_x_left_high, win_y_high), (0, 255, 0), 2)
        if right_current >= 0:
            cv2.rectangle(debug_img, (win_x_right_low, win_y_low), (win_x_right_high, win_y_high), (255, 0, 0), 2)

        if nz_pts.size > 0:
            mask_y = (nz_pts[:, 1] >= win_y_low) & (nz_pts[:, 1] < win_y_high)
            pts_win = nz_pts[mask_y]
            if left_current >= 0:
                mask_l = (pts_win[:, 0] >= win_x_left_low) & (pts_win[:, 0] < win_x_left_high)
                pts_l = pts_win[mask_l]
                if pts_l.size > 0:
                    left_current = int(np.mean(pts_l[:, 0]))
                    left_centers.append((left_current, 0.5 * (win_y_low + win_y_high)))
            if right_current >= 0:
                mask_r = (pts_win[:, 0] >= win_x_right_low) & (pts_win[:, 0] < win_x_right_high)
                pts_r = pts_win[mask_r]
                if pts_r.size > 0:
                    right_current = int(np.mean(pts_r[:, 0]))
                    right_centers.append((right_current, 0.5 * (win_y_low + win_y_high)))

    for c in left_centers:
        cv2.circle(debug_img, (int(c[0]), int(c[1])), 4, (0, 255, 255), -1)
    for c in right_centers:
        cv2.circle(debug_img, (int(c[0]), int(c[1])), 4, (0, 255, 255), -1)

    if left_centers and right_centers:
        n = min(len(left_centers), len(right_centers))
        mids = []
        for i in range(n):
            midx = 0.5 * (left_centers[i][0] + right_centers[i][0])
            midy = 0.5 * (left_centers[i][1] + right_centers[i][1])
            mids.append((midx, midy))
            cv2.circle(debug_img, (int(midx), int(midy)), 3, (0, 0, 255), -1)
        avg_mid_x = sum(m[0] for m in mids) / len(mids)
        cv2.line(debug_img, (int(avg_mid_x), h), (int(avg_mid_x), h - 80), (0, 0, 255), 2)

    return debug_img


def main():
    img_path = "/root/autorace_kkk_ws/frame_0001.png"
    if len(sys.argv) > 1:
        img_path = sys.argv[1]

    bgr = cv2.imread(img_path, cv2.IMREAD_COLOR)
    if bgr is None:
        print(f"failed to load image: {img_path}")
        sys.exit(1)

    h, w = bgr.shape[:2]
    roi_poly = make_roi_polygon(h, w)

    src_vis = bgr.copy()
    cv2.polylines(src_vis, [roi_poly.astype(np.int32)], isClosed=True, color=(0, 0, 255), thickness=2)

    bev = warp_to_bev(bgr, roi_poly)
    bev_binary = binarize_lanes(bev)
    debug_img = run_sliding_window(bev_binary)

    src_resized = cv2.resize(src_vis, (w, h))
    dbg_resized = cv2.resize(debug_img, (w, h))
    canvas = cv2.hconcat([src_resized, dbg_resized])

    # cv2.namedWindow("src+windows", cv2.WINDOW_NORMAL)
    # cv2.namedWindow("bev_binary", cv2.WINDOW_NORMAL)
    # cv2.imshow("src+windows", canvas)
    # cv2.imshow("bev_binary", bev_binary)
    cv2.waitKey(0)


if __name__ == "__main__":
    main()
