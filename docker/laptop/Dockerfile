FROM ros:jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-cv-bridge \
    python3-numpy \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY pacj_ws/src/px4_msgs src/px4_msgs
COPY pacj_ws/src/pacj src/pacj

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --packages-select px4_msgs pacj

COPY docker/laptop/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "pacj", "laptop.launch.py"]
