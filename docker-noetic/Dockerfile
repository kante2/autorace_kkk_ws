# ===============================
# ROS Noetic Dev Image (Desktop)
# ===============================
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm-256color \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8
    
# X11 + NVIDIA (OpenGL) friendly defaults
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    QT_X11_NO_MITSHM=1 \
    LIBGL_ALWAYS_INDIRECT=0
    
WORKDIR /root
COPY . /root/autorace_kkk_ws

# Common dev tools + ROS build helpers
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl vim tmux nano sudo \
    python3-pip python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    # Linear algebra / vision (C++ OpenCV)
    libeigen3-dev libopencv-dev \
    # Python OpenCV (cv2)
    python3-opencv \
    # PCL & friends (optional; safe to have)
    libpcl-dev ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-openslam-gmapping \
    # TF / common ROS deps
    ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs ros-noetic-nav-msgs \
    # --- MORAI에서 자주 쓰는 ROS 패키지 ---
    # 이미지/카메라 파이프라인
    ros-noetic-vision-opencv ros-noetic-cv-bridge ros-noetic-image-transport \
    ros-noetic-geographic-msgs ros-noetic-diagnostic-updater \ 
    ros-noetic-image-geometry ros-noetic-camera-info-manager ros-noetic-serial \
    ros-noetic-vision-msgs libgeographic-dev geographiclib-tools \
    # 주행/제어 메시지
    ros-noetic-sparse-bundle-adjustment ros-noetic-map-server \ 
    ros-noetic-ackermann-msgs \
    # 유틸 & 디버깅
    ros-noetic-rviz ros-noetic-rqt ros-noetic-rqt-graph ros-noetic-rqt-plot \
    # Video I/O (카메라/동영상 스트림 편의)
    v4l-utils ffmpeg \
    gstreamer1.0-tools gstreamer1.0-libav \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    # Utilities
    udev usbutils less htop \
    # ssh
    tzdata locales curl gnupg2 lsb-release ca-certificates \
    iproute2 net-tools iputils-ping netcat \
    openssh-client wget vim less git nano pip \
    mesa-utils libgl1-mesa-glx libglu1-mesa \
 && rm -rf /var/lib/apt/lists/*

# (선택) Python 유틸 패키지 몇 개
RUN python3 -m pip install --no-cache-dir \
    numpy matplotlib tqdm

# ROS setup + nice defaults
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc 