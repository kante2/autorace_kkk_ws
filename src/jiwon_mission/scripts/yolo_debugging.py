#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import os
from ultralytics import YOLO


# ===== 설정 =====
# YOLO weight 경로 (필요하면 여기 best.pt로 바꿔도 됨)
WEIGHT_PATH = "yolov8n.pt"   # 또는 "/root/autorace_kkk_ws/src/yolo/best.pt"

# 테스트할 이미지 경로
IMAGE_PATH = "/root/autorace_kkk_ws/src/yolo/autorace_left.jpg"

# confidence threshold
CONF_THRES = 0.5


def main():
    # 이미지 존재 여부 확인
    if not os.path.exists(IMAGE_PATH):
        print("[ERROR] 이미지 파일을 찾을 수 없습니다:", IMAGE_PATH)
        return

    # 이미지 읽기
    img = cv2.imread(IMAGE_PATH)
    if img is None:
        print("[ERROR] cv2.imread 실패:", IMAGE_PATH)
        return

    print("[INFO] 이미지 로드 완료:", IMAGE_PATH)
    print("[INFO] YOLO 모델 로딩 중:", WEIGHT_PATH)

    # YOLO 모델 로드
    model = YOLO(WEIGHT_PATH)

    # 추론
    results = model(img, conf=CONF_THRES, verbose=False)

    # 결과 그리기
    if len(results) > 0:
        r = results[0]

        if r.boxes is not None and len(r.boxes) > 0:
            boxes = r.boxes.xyxy.cpu().numpy()
            scores = r.boxes.conf.cpu().numpy()
            class_ids = r.boxes.cls.cpu().numpy().astype(int)

            names = model.names if hasattr(model, "names") else None

            for box, score, cls_id in zip(boxes, scores, class_ids):
                x1, y1, x2, y2 = box.astype(int)

                label = str(cls_id)
                if names is not None and cls_id in names:
                    label = names[cls_id]

                text = "{} {:.2f}".format(label, float(score))

                # 박스 & 텍스트 그리기
                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(
                    img,
                    text,
                    (x1, max(y1 - 5, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 0, 0),
                    2,
                )

    # 결과 창 띄우기
    cv2.namedWindow("yolo_result", cv2.WINDOW_NORMAL)
    cv2.imshow("yolo_result", img)
    print("[INFO] 아무 키나 누르면 창이 닫힙니다.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
