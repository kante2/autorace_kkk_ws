#!/usr/bin/env python3

import cv2
import glob
import os
from ultralytics import YOLO

model = YOLO("/root/autorace_kkk_ws/src/yolo/best.pt")

image_dir = "/root/autorace_kkk_ws/src/jiwon_mission/data"
image_paths = ["/root/autorace_kkk_ws/src/jiwon_mission/data/right_new_30.png"]

if not image_paths:
    raise SystemExit("이미지를 찾지 못했습니다. 경로를 확인해 주세요.")
else:
    print(f"Found {len(image_paths)} image(s).")

# 창 크기 조절 가능하게
cv2.namedWindow("YOLO result", cv2.WINDOW_NORMAL)
cv2.resizeWindow("YOLO result", 800, 600)   # 원하는 크기로

for img_path in image_paths:
    img = cv2.imread(img_path)
    if img is None:
        print(f"[WARN] 이미지 로드 실패: {img_path}")
        continue

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

    # <- 여기서 화면용으로만 축소해서 보여주기
    h, w = img.shape[:2]
    scale = 0.5  # 0.5면 가로/세로 모두 절반
    display = cv2.resize(img, (int(w * scale), int(h * scale)))

    window_name = os.path.basename(img_path)
    cv2.imshow("YOLO result", display)

    key = cv2.waitKey(0)
    cv2.destroyAllWindows()

cv2.destroyAllWindows()
