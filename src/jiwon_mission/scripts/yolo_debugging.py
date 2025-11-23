#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import os
from ultralytics import YOLO

# 1) 모델 로드
model = YOLO("/root/autorace_kkk_ws/src/yolo/best.pt")

# 2) 한 장만 테스트할 이미지 경로
img_path = "/root/autorace_kkk_ws/src/jiwon_mission/data/knate_test1.png"

if not os.path.isfile(img_path):
    raise SystemExit(f"이미지를 찾지 못했습니다: {img_path}")

print(f"Test image: {img_path}")

# 3) 이미지 로드
img = cv2.imread(img_path)
if img is None:
    raise SystemExit(f"[WARN] 이미지 로드 실패: {img_path}")

# 4) YOLO 추론
results = model(img, conf=0.5)[0]

for box in results.boxes:
    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
    cls_id = int(box.cls)
    conf = float(box.conf)
    cls_name = model.names[cls_id] if model.names else str(cls_id)

    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    label = f"{cls_name} {conf:.2f}"
    cv2.putText(img, label, (x1, max(y1 - 5, 15)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

# 5) 보기 좋게 축소해서 보여주기
h, w = img.shape[:2]
scale = 0.5
display = cv2.resize(img, (int(w * scale), int(h * scale)))

cv2.namedWindow("YOLO result", cv2.WINDOW_NORMAL)
cv2.resizeWindow("YOLO result", 800, 600)
cv2.imshow("YOLO result", display)

cv2.waitKey(0)
cv2.destroyAllWindows()
