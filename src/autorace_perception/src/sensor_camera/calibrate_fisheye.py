# calibrate_fisheye.py
# 저장한 이미지들을 읽어 cv.fisheye.calibrate 로 K(내부행렬), D(4계수 왜곡)을 구합니다.
# 결과를 calib_fisheye.npz로 저장합니다.

import cv2 as cv
import numpy as np
import glob
import os

# === 사용자가 맞춰야 하는 값 ===
CHECKERBOARD = (9, 6)   # 내부 코너 수 (가로, 세로)
SQUARE_SIZE = 0.025     # 정사각형 한 칸 실제 길이(미터). 단위만 일관되면 됨.
IMG_DIR = "data"        # 1단계에서 저장한 폴더
SAVE_PATH = "calib_fisheye.npz"

# 체커보드 3D 포인트 (Z=0 평면)
objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float64)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []  # 각 이미지마다 objp 복사
imgpoints = []  # 각 이미지마다 코너 (Nx1x2)

images = sorted(glob.glob(os.path.join(IMG_DIR, "*.png")) + 
                glob.glob(os.path.join(IMG_DIR, "*.jpg")) + 
                glob.glob(os.path.join(IMG_DIR, "*.jpeg")))

assert len(images) > 0, "캘리브레이션용 이미지가 없습니다."

imsize = None
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 1e-6)

good = 0
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    if imsize is None:
        imsize = gray.shape[::-1]  # (W, H)

    ret, corners = cv.findChessboardCorners(
        gray, CHECKERBOARD,
        flags=cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE
    )
    if ret:
        corners_refined = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners_refined.reshape(1, -1, 2))
        good += 1

print(f"[INFO] 감지 성공 이미지: {good}/{len(images)}")
assert good >= 8, "감지 성공 이미지가 너무 적습니다. 더 많이 찍어주세요."

K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs, tvecs = [], []

# fisheye 전용 플래그 (권장)
flags = (cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
         cv.fisheye.CALIB_CHECK_COND |
         cv.fisheye.CALIB_FIX_SKEW)

rms, K, D, rvecs, tvecs = cv.fisheye.calibrate(
    objectPoints=objpoints,
    imagePoints=imgpoints,
    image_size=imsize,
    K=K, D=D, rvecs=rvecs, tvecs=tvecs,
    flags=flags,
    criteria=criteria
)

print(f"[RESULT] RMS={rms:.6f}")
print("[RESULT] K=\n", K)
print("[RESULT] D=\n", D)

np.savez(SAVE_PATH, K=K, D=D, rms=rms, imsize=np.array(imsize))
print(f"[SAVED] {SAVE_PATH}")
