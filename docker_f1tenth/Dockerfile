FROM osrf/ros:foxy-desktop
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

# 필수/자주 쓰는 도구 + ROS2 빌드/런타임 의존 + Gazebo + GUI 확인 툴
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3-pip \
    python3-colcon-common-extensions \
    mesa-utils x11-apps nano \
    ros-foxy-xacro \
    ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-hardware-interface ros-foxy-pluginlib \
    ros-foxy-gazebo-ros-pkgs ros-foxy-gazebo-ros2-control \
    ros-foxy-joint-state-publisher ros-foxy-joint-state-publisher-gui ros-foxy-robot-state-publisher \
    ros-foxy-teleop-twist-keyboard \
    ros-foxy-geographic-msgs ros-foxy-nav2-map-server \
    && rm -rf /var/lib/apt/lists/*

# key_teleop용 evdev (파이썬 모듈)
RUN pip3 install --no-cache-dir evdev || true

# 컨테이너 접속 시 자동으로 ROS 환경 로드
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
WORKDIR /root
