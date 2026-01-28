# Stage 1: UR10 simulation with ROS 2 Jazzy + Gazebo Harmonic
# Base: Ubuntu 24.04 (Noble); no ROS/Gazebo on host required
FROM ubuntu:24.04

ENV LANG=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

# Locale
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# System and ROS 2 repo
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS 2 Jazzy + Gazebo Harmonic + gz_ros2_control + UR description
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-ur-description \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-gz-ros2-control-demos \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-image \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    mesa-utils \
    libgl1-mesa-dri \
    libglx-mesa0 \
    mesa-vulkan-drivers \
    && rm -rf /var/lib/apt/lists/*

# Non-root user and workspace (UID 1001; 1000 often taken in base image)
RUN useradd -m -u 1001 -s /bin/bash ros \
    && mkdir -p /home/ros/ws/src \
    && chown -R ros:ros /home/ros

# Copy workspace source (will be overwritten by compose mount or build context)
COPY --chown=ros:ros src /home/ros/ws/src

# Entrypoint script (copied into container root for clarity)
COPY docker/entrypoint.sh /docker/entrypoint.sh
RUN chmod +x /docker/entrypoint.sh

WORKDIR /home/ros/ws

# Build workspace (run as root for rosdep apt; then chown for ros)
RUN . /opt/ros/jazzy/setup.sh \
    && rosdep update \
    && rosdep install -r -y --from-paths src --ignore-src || true \
    && colcon build --symlink-install \
    && chown -R ros:ros /home/ros

USER ros

# Default: source and launch
ENTRYPOINT ["/docker/entrypoint.sh"]
CMD ["ros2", "launch", "ur10_gz_bringup", "ur10_gz_bringup.launch.py"]
