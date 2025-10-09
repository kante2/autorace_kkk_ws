NVIDIA DOCKER TOOLKIT


# X11 허용
xhost +local:docker

# 빌드
cd ~/autorace_kkk_ws/docker-noetic

docker compose up -d --build

