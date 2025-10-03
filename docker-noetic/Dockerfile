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
    # Linear algebra / vision / geo
    libeigen3-dev libopencv-dev libgeographic-dev geographiclib-tools \
    # PCL & friends (optional; safe to have)
    libpcl-dev ros-noetic-pcl-ros ros-noetic-pcl-conversions \
    # TF / common ROS deps
    ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs ros-noetic-nav-msgs \
    # Utilities
    udev usbutils less htop \
 && rm -rf /var/lib/apt/lists/*

# Ensure GeographicLib datasets are present (for GeographicLib::LocalCartesian accuracy)
RUN geographiclib-get-geoids egm96-5 && geographiclib-get-datasets gravity magnetic

# Create a non-root user (optional). Comment out if you prefer root.
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

# Workspace mount point hint (will be bind-mounted by docker-compose)
WORKDIR /root/autorace_kkk_ws

# Default shell
USER ${USER}
CMD ["/bin/bash"]
