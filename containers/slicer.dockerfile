# Slicer + SlicerROS2 + SlicerFranka container.
#
# ROS Jazzy desktop, Qt5 dev libs, Slicer v5.10.0 SuperBuild, SlicerROS2.
#
# Build (from repo root):
#   docker build \
#     -f containers/slicer.dockerfile \
#     -t slicerfranka-slicer:local .
#
# See containers/run-slicer.sh for the recommended `docker run` invocation.

FROM ubuntu:24.04

ARG ROS_DISTRO=jazzy
ARG SLICER_VERSION=v5.10.0
ARG SLICER_ROS2_VERSION=v1.2

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    QT_X11_NO_MITSHM=1

SHELL ["/bin/bash", "-c"]

# --- system base + locale ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales tzdata sudo curl git git-lfs ca-certificates gnupg \
        lsb-release software-properties-common \
        build-essential cmake cmake-curses-gui pkg-config patch \
        tilix \
 && locale-gen en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*

# --- ROS 2 Jazzy ---
RUN add-apt-repository universe \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
        > /etc/apt/sources.list.d/ros2.list \
 && apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-ament-cmake \
        ros-${ROS_DISTRO}-object-recognition-msgs \
        ros-${ROS_DISTRO}-moveit-msgs \
        ros-${ROS_DISTRO}-moveit-core \
        ros-${ROS_DISTRO}-moveit-ros-planning \
        ros-${ROS_DISTRO}-moveit-ros-planning-interface \
        python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# --- Slicer Qt5 build deps ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        libqt5x11extras5-dev qtmultimedia5-dev libqt5svg5-dev \
        qtwebengine5-dev libqt5xmlpatterns5-dev qttools5-dev \
        qtbase5-private-dev qtbase5-dev qt5-qmake \
        libxt-dev libssl-dev libbz2-dev libglu1-mesa-dev libnss3 \
        libpulse-mainloop-glib0 libasound2t64 \
 && rm -rf /var/lib/apt/lists/*

# --- user setup ---
# Ubuntu 24.04 base images ship with a default `ubuntu` user at UID 1000.
# We reuse it as-is and just grant sudo.
RUN echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu \
 && chmod 0440 /etc/sudoers.d/ubuntu \
 && chsh -s /bin/bash ubuntu

USER ubuntu
WORKDIR /home/ubuntu
RUN mkdir -p /home/ubuntu/.config/NA-MIC

# --- Slicer v5.10.0 SuperBuild (the long step, potentially several hours) ---
RUN mkdir -p /home/ubuntu/slicer \
 && cd /home/ubuntu/slicer \
 && git clone --depth 1 --branch ${SLICER_VERSION} \
        https://github.com/Slicer/Slicer.git

WORKDIR /home/ubuntu/slicer/Slicer-SuperBuild
RUN cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DSlicer_USE_SYSTEM_OpenSSL=ON \
        -DSlicer_USE_SYSTEM_bzip2=ON \
        -DSlicer_BUILD_TESTING=OFF \
        -DBUILD_TESTING=OFF \
        -DSlicer_DOWNLOAD_TEST_DATA=OFF \
        -DSlicer_USE_TESTING_DATA=OFF \
        ../Slicer \
 && JOBS=$(( $(nproc) - 4 )) \
 && if [ "$JOBS" -lt 1 ]; then JOBS=1; fi \
 && make -j"$JOBS"

# --- SlicerROS2 (colcon workspace, built against the Slicer above) ---
WORKDIR /home/ubuntu/ros2_ws/src
RUN git clone --depth 1 --branch ${SLICER_ROS2_VERSION} \
        https://github.com/rosmed/slicer_ros2_module

WORKDIR /home/ubuntu/ros2_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build \
        --packages-select slicer_ros2_module \
        --cmake-args \
            -DSlicer_DIR=/home/ubuntu/slicer/Slicer-SuperBuild/Slicer-build \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_TESTING=OFF

# --- aliases + auto-source ROS in interactive shells ---
RUN { \
        echo 'source /opt/ros/'"${ROS_DISTRO}"'/setup.bash'; \
        echo '[ -f /home/ubuntu/ros2_ws/install/setup.bash ] && source /home/ubuntu/ros2_ws/install/setup.bash'; \
        echo 'alias slicer="ros2 run slicer_ros2_module slicer --additional-module-paths /home/ubuntu/SlicerFranka"'; \
        echo 'alias ll="ls -lah"'; \
    } >> /home/ubuntu/.bashrc

USER root
COPY containers/slicer-entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
USER ubuntu

WORKDIR /home/ubuntu
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
