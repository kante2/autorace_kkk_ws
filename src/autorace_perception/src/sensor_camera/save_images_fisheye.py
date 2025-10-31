# save_images_fisheye.py
# 웹캠(또는 동영상)에서 프레임을 받아 체커보드를 인식하면 초록색으로 표시됩니다.
# s 를 누르면 현재 프레임을 data/ 폴더에 저장합니다.
# 체커보드는 내부 코너 개수(가로×세로)를 정확히 넣어야 합니다. 예: 9×6

import cv2 as cv
import os
from datetime import datetime

# === 사용자가 맞춰야 하는 값 ===
CHECKERBOARD = (9, 6)  # 내부 코너 수 (가로, 세로)
CAM_INDEX = 0          # 카메라 인덱스
SAVE_DIR = "data"      # 저장 폴더

os.makedirs(SAVE_DIR, exist_ok=True)
cap = cv.VideoCapture(CAM_INDEX)

if not cap.isOpened():
    raise RuntimeError("카메라를 열 수 없습니다. CAM_INDEX를 확인하세요.")

print("[INFO] 's'로 저장, 'q'로 종료")
while True:
    ok, frame = cap.read()
    if not ok:
        break

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(
        gray, CHECKERBOARD,
        flags=cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK + cv.CALIB_CB_NORMALIZE_IMAGE
    )

    vis = frame.copy()
    if ret:
        # 정밀 코너
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        cv.drawChessboardCorners(vis, CHECKERBOARD, corners2, ret)
        cv.putText(vis, "DETECTED", (20, 40), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    else:
        cv.putText(vis, "NOT FOUND", (20, 40), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv.imshow("Capture for Fisheye Calibration", vis)
    key = cv.waitKey(1) & 0xFF
    if key == ord('s'):
        fname = os.path.join(SAVE_DIR, f"img_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.png")
        cv.imwrite(fname, frame)
        print(f"[SAVED] {fname}")
    elif key == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
