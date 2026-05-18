# Franka controller container.
#
# Bakes in: ROS Jazzy base, libfranka 0.9.2, TOPP-RA C++, Eigen, yaml-cpp,
# build toolchain. franka_controller and franka_description are
# bind-mounted at run time and built by the entrypoint on first launch.
#
# Build (from repo root):
#   docker build \
#     --build-arg UID=$(id -u) --build-arg GID=$(id -g) \
#     -f containers/controller.dockerfile \
#     -t slicerfranka-controller:local .
#
# Or pull the prebuilt image:
#   docker pull ghcr.io/iselein/slicerfranka-controller:0.9.2-jazzy
#
# See containers/run-controller.sh for the recommended `docker run` invocation
# (RT capabilities, host networking).

FROM ubuntu:24.04

ARG ROS_DISTRO=jazzy
ARG LIBFRANKA_VERSION=0.9.2

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

SHELL ["/bin/bash", "-c"]

# --- system base + locale ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales tzdata sudo curl git ca-certificates gnupg \
        lsb-release software-properties-common \
        build-essential cmake pkg-config \
 && locale-gen en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*

# --- ROS 2 Jazzy (base, no desktop — controller is headless) ---
RUN add-apt-repository universe \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
        > /etc/apt/sources.list.d/ros2.list \
 && apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-ament-cmake \
        ros-${ROS_DISTRO}-tf2-ros \
        python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# --- libfranka build deps + shared math/yaml libs (used by franka_controller too) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        libpoco-dev libeigen3-dev libfmt-dev libyaml-cpp-dev \
 && rm -rf /var/lib/apt/lists/*

# --- libfranka from source ---
WORKDIR /opt
RUN git clone --recursive --branch ${LIBFRANKA_VERSION} \
        https://github.com/frankaemika/libfranka
RUN mkdir -p /opt/libfranka/build
WORKDIR /opt/libfranka/build
RUN cmake -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF .. \
 && make -j$(nproc) \
 && make install \
 && ldconfig

# --- TOPP-RA (C++ only, no pinocchio dep) ---
WORKDIR /opt
RUN git clone https://github.com/hungpham2511/toppra
RUN mkdir -p /opt/toppra/cpp/build
WORKDIR /opt/toppra/cpp/build
RUN cmake -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_WITH_PINOCCHIO=OFF .. \
 && make -j$(nproc) \
 && make install \
 && ldconfig

# --- user setup ---
# Ubuntu 24.04 base images ship with a default `ubuntu` user at UID 1000.
# We reuse it as-is and just grant sudo.
RUN echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu \
 && chmod 0440 /etc/sudoers.d/ubuntu \
 && chsh -s /bin/bash ubuntu

USER ubuntu
WORKDIR /home/ubuntu
RUN mkdir -p /home/ubuntu/ros2_ws/src

# --- aliases + auto-source ROS in interactive shells ---
RUN { \
        echo 'source /opt/ros/'"${ROS_DISTRO}"'/setup.bash'; \
        echo '[ -f /home/ubuntu/ros2_ws/install/setup.bash ] && source /home/ubuntu/ros2_ws/install/setup.bash'; \
        echo 'alias ctlr="ros2 launch franka_controller controller.launch.py"'; \
        echo 'alias ll="ls -lah"'; \
    } >> /home/ubuntu/.bashrc

USER root
COPY containers/controller-entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
USER ubuntu

WORKDIR /home/ubuntu
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
