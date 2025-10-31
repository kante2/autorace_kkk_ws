fisheyeview인 카메라 센서의 왜곡을 피기 위한 패키지
25/10/29 ~ 25/10/31 작업



사용법

# 설치
pip install opencv-python pyyaml

# 예: 내부 코너 9x6, 한 칸 24mm, PNG 이미지 폴더
python calibrate_chessboard.py \
  --images "data/*.png" \
  --cols 9 --rows 6 \
  --square 0.024 \
  --save calibration.yaml

# 어안(Equidistant) 모델로 보정 파라미터 추정
python calibrate_chessboard.py \
  --images "fisheye/*.jpg" \
  --cols 9 --rows 6 \
  --square 0.024 \
  --fisheye \
  --save calibration_fisheye.yaml


