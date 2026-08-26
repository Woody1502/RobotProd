FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-shapely \
    python3-future \
    # Localization
    ros-jazzy-robot-localization \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    # Sensors & drivers
    ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher \
    # TF
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-xacro \
    # DDS
    ros-jazzy-rmw-cyclonedds-cpp \
    # Camera / vision
    ros-jazzy-image-transport \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-geometry \
    ros-jazzy-tf2-geometry-msgs \
    python3-opencv \
    python3-scipy \
    python3-matplotlib \
    # RS-485
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install minimalmodbus pynput --break-system-packages

RUN apt-get update && apt-get install -y socat && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY src/ src/

RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

COPY docker_entrypoint.sh /docker_entrypoint.sh
RUN chmod +x /docker_entrypoint.sh

ENTRYPOINT ["/docker_entrypoint.sh"]
CMD ["bash"]
