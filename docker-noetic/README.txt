# autorace_kkk_ws - ROS Noetic Docker Dev

이 디렉토리(`/home/kante/autorace_kkk_ws/docker-noetic`)에서 아래 파일로 컨테이너를 빌드/실행합니다.
- `Dockerfile`
- `docker-compose.yaml`
- `README.txt`

## 요구사항
- NVIDIA 드라이버 (옵션: GPU 가속 사용할 경우)
- Docker + docker-compose
- X11 (리눅스 데스크탑; RViz, rqt 등 GUI 실행용)

## 0) X11 허용
```bash
xhost +local:docker
```

## 1) 빌드
```bash
cd ~/autorace_kkk_ws/docker-noetic
docker compose build \
  --build-arg UID=$(id -u) \
  --build-arg GID=$(id -g)
```

> GPU 없는 환경이면 `docker-compose.yaml`의 `runtime: nvidia`와 `NVIDIA_*` env를 제거하거나 주석 처리하세요.

## 2) 실행
```bash
# DISPLAY 전달 필요 (일반적인 리눅스 환경은 자동)
export DISPLAY=${DISPLAY}

# 컨테이너 실행
docker compose up -d
# 접속
docker exec -it autorace-noetic-dev bash
```

## 3) ROS/워크스페이스
컨테이너 안에서 기본 경로:
```
/root/autorace_kkk_ws
```
호스트의 `~/autorace_kkk_ws`가 마운트되어 있습니다.

처음 한 번만(의존성 설치 자동화용):
```bash
# 컨테이너 내부
sudo rosdep init || true
rosdep update
# src에 있는 패키지들에 대해 의존성 설치
rosdep install --from-paths src --ignore-src -r -y
```

### Catkin 빌드 예시 (ROS1 Noetic)
```bash
source /opt/ros/noetic/setup.bash
cd /root/autorace_kkk_ws
catkin_make
source devel/setup.bash
```

## 4) GUI 앱 (RViz 등)
컨테이너 안에서:
```bash
rviz
rqt
```

## 5) MORAI
`morai_msgs`를 소스에 두었다면 catkin 빌드 시 함께 컴파일됩니다. apt 패키지가 없을 수 있으므로 소스 추가 후 `rosdep install`을 사용하세요.

## 6) 시리얼/USB 장치
`privileged: true`와 `/dev` 바인드로 대부분의 장치 접근이 가능합니다. 추가 udev 룰이 필요하면 호스트에 설정하거나 컨테이너에 추가하세요.

## 7) 문제 해결
- GUI가 안 뜨면:
  - `xhost +local:docker`
  - `DISPLAY`가 비어있지 않은지 확인
  - NVIDIA 환경이면 호스트에 nvidia-container-toolkit 설치 확인
- 파일 소유권 문제:
  - `docker compose build` 시 `UID/GID`를 본인 계정으로 전달했는지 확인
- 네트워킹(ROS master 공유):
  - 기본은 `network_mode: host`라 로컬 ROS Master 공유가 쉽습니다.
  - 여러 PC 간 통신은 `ROS_MASTER_URI`, `ROS_IP/ROS_HOSTNAME`을 알맞게 설정하세요.

## 8) 흔한 환경변수
```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=$(hostname -I | awk '{print $1}')
```

## 9) 종료/정리
```bash
docker compose down
```

---
**Mount 경로**: 호스트 `~/autorace_kkk_ws` ↔ 컨테이너 `/root/autorace_kkk_ws`  
원하면 `docker-compose.yaml`의 `volumes` 섹션을 수정하세요.
