# ===============================
# ROS Noetic Dev Image (Desktop)
# ===============================
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm-256color \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

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
    libpcl-dev ros-noetic-pcl-ros ros-noetic-pcl-conversions \
    # TF / common ROS deps
    ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs ros-noetic-nav-msgs \
    # --- MORAI에서 자주 쓰는 ROS 패키지 ---
    # 이미지/카메라 파이프라인
    ros-noetic-vision-opencv ros-noetic-cv-bridge ros-noetic-image-transport \
    ros-noetic-image-geometry ros-noetic-camera-info-manager \
    # 주행/제어 메시지
    ros-noetic-ackermann-msgs \
    # 유틸 & 디버깅
    ros-noetic-rviz ros-noetic-rqt ros-noetic-rqt-graph ros-noetic-rqt-plot \
    # Video I/O (카메라/동영상 스트림 편의)
    v4l-utils ffmpeg \
    gstreamer1.0-tools gstreamer1.0-libav \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    # Utilities
    udev usbutils less htop \
 && rm -rf /var/lib/apt/lists/*

# (GeographicLib datasets 설치 제거됨)

# Create a non-root user
ARG USER=developer
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} ${USER} && \
    useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USER} && \
    usermod -aG sudo ${USER} && \
    echo "${USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER}

# X11 + NVIDIA (OpenGL) friendly defaults
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    QT_X11_NO_MITSHM=1 \
    LIBGL_ALWAYS_INDIRECT=0

# ROS setup + nice defaults
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc && \
    echo "export ROS_DISTRO=noetic" >> /etc/bash.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=0" >> /etc/bash.bashrc

# (선택) Python 유틸 패키지 몇 개
RUN python3 -m pip install --no-cache-dir \
    numpy matplotlib tqdm

# Workspace mount point hint (home 아래)
WORKDIR /home/${USER}/autorace_kkk_ws

# Default shell
USER ${USER}
CMD ["/bin/bash"]

