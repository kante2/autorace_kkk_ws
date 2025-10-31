#!/usr/bin/env python3
# calibrate_chessboard.py
import argparse, glob, os, sys, yaml
import numpy as np
import cv2

def parse_args():
    ap = argparse.ArgumentParser(description="Mono camera calibration with chessboard images")
    ap.add_argument("--images", type=str, required=True,
                    help="Glob for images, e.g. 'data/*.png'")
    ap.add_argument("--cols", type=int, required=True,
                    help="Number of inner corners across columns (e.g. 9)")
    ap.add_argument("--rows", type=int, required=True,
                    help="Number of inner corners across rows (e.g. 6)")
    ap.add_argument("--square", type=float, default=0.024,
                    help="Square size in meters (used for scale; any unit OK)")
    ap.add_argument("--fisheye", action="store_true",
                    help="Use OpenCV fisheye (equidistant) model")
    ap.add_argument("--save", type=str, default="calibration.yaml",
                    help="Output YAML path")
    return ap.parse_args()

def collect_points(img_paths, pattern_size, square, use_fisheye):
    # 3D chessboard points in board frame (Z=0 plane)
    objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square

    objpoints, imgpoints = [], []
    gray_shape = None

    # corner refine criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    for p in img_paths:
        img = cv2.imread(p, cv2.IMREAD_COLOR)
        if img is None:
            print(f"[WARN] failed to read: {p}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_shape = gray.shape[::-1]

        ok, corners = cv2.findChessboardCorners(gray, pattern_size,
                                                flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok and use_fisheye:
            # fisheye는 보정 전이라 코너 탐지가 어려울 수 있음 → 보정 try
            ok, corners = cv2.findChessboardCorners(gray, pattern_size)

        if ok:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp.copy())
            imgpoints.append(corners)
            print(f"[OK] corners: {os.path.basename(p)}")
        else:
            print(f"[X ] no corners: {os.path.basename(p)}")

    if len(objpoints) < 8:
        print("[ERR] Detected corners are too few (<8). Add more images with varied poses.")
        sys.exit(1)

    return objpoints, imgpoints, gray_shape

def calibrate_plumb_bob(objpoints, imgpoints, img_size):
    flag = (cv2.CALIB_RATIONAL_MODEL)  # 괜찮은 기본값
    ret, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None, flags=flag)
    # 평균 리프로젝션 에러
    tot_err = 0.0
    pts = 0
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, D)
        err = cv2.norm(imgpoints[i], proj, cv2.NORM_L2)
        tot_err += err*err
        pts += len(objpoints[i])
    rms = np.sqrt(tot_err/pts)
    return rms, K, D

def calibrate_fisheye(objpoints, imgpoints, img_size):
    # fisheye: D는 4계수, K는 3x3
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs, tvecs = [], []
    flag = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        objpoints, imgpoints, img_size, K, D, rvecs, tvecs, flag,
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))

    # 리프로젝션 에러 수동 계산
    tot_err = 0.0
    pts = 0
    for i in range(len(objpoints)):
        proj = cv2.fisheye.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, D)[0]
        err = cv2.norm(imgpoints[i], proj, cv2.NORM_L2)
        tot_err += err*err
        pts += len(objpoints[i])
    rms2 = np.sqrt(tot_err/pts)
    return float(rms2), K, D

def save_ros_yaml(path, K, D, img_size, fisheye):
    w, h = img_size
    # ROS camera_info 스타일
    data = {
        "image_width": int(w),
        "image_height": int(h),
        "camera_name": "calibrated_camera",
        "distortion_model": "equidistant" if fisheye else "plumb_bob",
        "distortion_coefficients": {
            "data": D.flatten(order='C').tolist()
        },
        "camera_matrix": {
            "data": K.flatten(order='C').tolist()
        },
        "rectification_matrix": {
            "data": np.eye(3, dtype=float).flatten(order='C').tolist()
        },
        "projection_matrix": {
            # 기본값: K를 P에 박음 (Tx=0)
            "data": np.hstack([K, np.zeros((3,1))]).flatten(order='C').tolist()
        }
    }
    with open(path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print(f"[SAVE] {path}")

def main():
    args = parse_args()
    img_paths = sorted(glob.glob(args.images))
    if not img_paths:
        print("[ERR] no images matched. e.g. --images 'data/*.jpg'")
        sys.exit(1)

    pattern = (args.cols, args.rows)
    objpoints, imgpoints, img_size = collect_points(img_paths, pattern, args.square, args.fisheye)

    if args.fisheye:
        rms, K, D = calibrate_fisheye(objpoints, imgpoints, img_size)
    else:
        rms, K, D = calibrate_plumb_bob(objpoints, imgpoints, img_size)

    print("\n=== Calibration Result ===")
    print(f"RMS reprojection error: {rms:.4f} px")
    print("K (camera matrix):\n", K)
    print("D (distortion coeffs):\n", D.ravel())

    save_ros_yaml(args.save, K, D, img_size, args.fisheye)

if __name__ == "__main__":
    main()
