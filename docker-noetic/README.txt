NVIDIA DOCKER TOOLKIT


# X11 허용
xhost +local:docker

# 빌드
cd ~/autorace_kkk_ws/docker-noetic
docker compose build --build-arg UID=$(id -u) --build-arg GID=$(id -g)

# 실행
docker compose up -d

# 마운트 확인(컨테이너 내부에서)
docker compose exec ros-noetic-dev bash -lc 'whoami && pwd && ls -al'

