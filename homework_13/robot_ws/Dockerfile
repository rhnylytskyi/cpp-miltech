FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/robot_ws

COPY src ./src

RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --packages-select antidrone_turret

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "antidrone_turret", "system.launch.py"]
