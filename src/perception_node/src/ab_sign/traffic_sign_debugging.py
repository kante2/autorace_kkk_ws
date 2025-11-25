#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
traffic_sign_debugging.py
  - YOLO로 바운딩 박스를 얻은 뒤, 박스 영역을 이진화하여
    상단 절반의 좌/우 흰색 픽셀 수를 비교해 LEFT/RIGHT를 판단.
  - ORB/SIFT 매칭 없이 단순 픽셀 카운트로 동작.
"""
import os
import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

# simple blue filter (HSV)
BLUE_LOWER = (100, 80, 50)
BLUE_UPPER = (140, 255, 255)
BLUE_RATIO_MIN = 0.02  # 최소 파란색 비율(2%) 미만이면 스킵

def load_bgr(path):
    if not os.path.exists(path):
        raise IOError("file not found: {}".format(path))
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        raise IOError("failed to read image: {}".format(path))
    return img


def main():
    base = "/root/autorace_kkk_ws/src/perception_node/src/ab_sign"
    real_path = os.path.join(base, "30.png")

    real_bgr = load_bgr(real_path)
    real_gray_full = cv2.cvtColor(real_bgr, cv2.COLOR_BGR2GRAY)

    # YOLO crop settings
    yolo_model_path = os.path.join(base, "best_crop.pt")
    yolo_conf = 0.25
    crop_margin = 12

    crops = []
    if YOLO is not None and os.path.exists(yolo_model_path):
        try:
            model = YOLO(yolo_model_path)
            results = model(real_bgr, verbose=False, conf=yolo_conf)
            h, w = real_gray_full.shape[:2]
            for res in results:
                if not hasattr(res, "boxes") or res.boxes is None:
                    continue
                names = res.names if hasattr(res, "names") else {}
                for i, box in enumerate(res.boxes.xyxy.cpu().numpy()):
                    cls_id = int(res.boxes.cls[i].item()) if res.boxes.cls is not None else -1
                    cls_name = names.get(cls_id, str(cls_id))
                    x1, y1, x2, y2 = box
                    x1 = int(max(0, x1 - crop_margin))
                    y1 = int(max(0, y1 - crop_margin))
                    x2 = int(min(w - 1, x2 + crop_margin))
                    y2 = int(min(h - 1, y2 + crop_margin))
                    if x2 <= x1 or y2 <= y1:
                        continue
                    roi_bgr = real_bgr[y1:y2, x1:x2].copy()
                    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
                    mask_blue = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
                    blue_ratio = float(cv2.countNonZero(mask_blue)) / float(max(mask_blue.size, 1))
                    if blue_ratio < BLUE_RATIO_MIN:
                        print(f"Skip box {cls_name} due to low blue ratio ({blue_ratio:.3f})")
                        continue
                    roi_gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
                    crops.append((roi_gray, (x1, y1), (x2, y2), cls_name))
                    cv2.rectangle(real_bgr, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    label = f"{cls_name} B{blue_ratio:.2f}"
                    cv2.putText(real_bgr, label, (x1, max(y1 - 5, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            if not crops:
                print("YOLO found no boxes; no crops to run ORB on.")
        except Exception as e:
            print("YOLO inference failed, fallback to full frame:", e)
    else:
        if YOLO is None:
            print("ultralytics not installed; skipping YOLO crop.")
        else:
            print("YOLO model not found at:", yolo_model_path)

    if not crops:
        cv2.putText(real_bgr, "NO YOLO BOX", (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 0, 255), 2)
    else:
        for crop_gray, (ox, oy), (x2, y2), cls_name in crops:
            # 이진화 (Otsu)
            _, bin_roi = cv2.threshold(crop_gray, 0, 255,
                                       cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            h_roi, w_roi = bin_roi.shape[:2]
            top = bin_roi[:max(1, h_roi // 2), :]
            mid = w_roi // 2
            left_count = cv2.countNonZero(top[:, :mid])
            right_count = cv2.countNonZero(top[:, mid:])
            decision = "LEFT" if left_count > right_count else "RIGHT"
            print(f"ROI ({ox},{oy})-({x2},{y2}) cls={cls_name} left={left_count} right={right_count} -> {decision}")
            info = f"{decision} L:{left_count} R:{right_count} cls:{cls_name}"
            cv2.putText(real_bgr, info, (ox, max(oy - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("real", real_bgr)

    # Graceful exit: press ESC/q or close window
    while True:
        key = cv2.waitKey(30) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            break
        # If windows are closed manually, exit loop
        if cv2.getWindowProperty("real", cv2.WND_PROP_VISIBLE) < 1:
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
