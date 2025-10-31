# preview_undistort_fisheye.py
# 저장한 K, D로 실시간으로 평탄화가 잘 되는지 확인합니다.

# 트랙바의 balance(0~100)를 움직여 시야/크롭 정도를 조절합니다.

# 0: 왜곡은 잘 펴지지만 더 많이 크롭됨

# 100: 더 넓게 보이지만 가장자리 왜곡/블랙영역이 일부 보일 수 있음

import cv2 as cv
import numpy as np

CALIB_FILE = "calib_fisheye.npz"
CAM_INDEX = 0

data = np.load(CALIB_FILE, allow_pickle=True)
K = data["K"]
D = data["D"]
imsize = tuple(data["imsize"])  # (W, H)
print("[LOAD] K=\n", K)
print("[LOAD] D=\n", D.reshape(-1))
print("[LOAD] imsize=", imsize)

cap = cv.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise RuntimeError("카메라를 열 수 없습니다.")

def build_maps(balance_val):
    # balance: 0~1.0
    balance = balance_val / 100.0
    # 새로운 카메라 행렬 Knew 추정 (사실상 시야/크롭 조절)
    Knew = cv.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, imsize, np.eye(3), balance=balance, fov_scale=1.0
    )
    map1, map2 = cv.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), Knew, imsize, cv.CV_16SC2
    )
    return map1, map2

cv.namedWindow("Fisheye Undistort Preview", cv.WINDOW_NORMAL)
cv.resizeWindow("Fisheye Undistort Preview", 1280, 720)
cv.createTrackbar("balance(0~100)", "Fisheye Undistort Preview", 0, 100, lambda v: None)

map1, map2 = build_maps(cv.getTrackbarPos("balance(0~100)", "Fisheye Undistort Preview"))

print("[INFO] 'q' 종료. 트랙바로 balance 조절.")
while True:
    ok, frame = cap.read()
    if not ok:
        break

    # 입력 프레임 크기가 캘리브레이션 크기와 다르면 리사이즈(권장X, 가능하면 같은 해상도에서 보정)
    h, w = frame.shape[:2]
    if (w, h) != imsize:
        frame = cv.resize(frame, imsize, interpolation=cv.INTER_AREA)

    # 트랙바 변경 체크
    bal_val = cv.getTrackbarPos("balance(0~100)", "Fisheye Undistort Preview")
    if 'prev_bal' not in globals() or prev_bal != bal_val:
        map1, map2 = build_maps(bal_val)
        prev_bal = bal_val

    undist = cv.remap(frame, map1, map2, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)

    # 비교 보기
    side = np.hstack([frame, undist])
    cv.imshow("Fisheye Undistort Preview", side)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
