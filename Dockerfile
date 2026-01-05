FROM unfrobotics/docker-ros2-jazzy-gz-rviz2:latest

ENV DEBIAN_FRONTEND=noninteractive

RUN set -eux; \
    find /etc/apt -type f \( -name "*.list" -o -name "*.sources" \) \
    -exec grep -l "packages.ros.org" {} + \
    | tee /tmp/removed_ros_sources.txt \
    | xargs -r rm -f; \
    echo "Removed ROS apt source files:"; \
    cat /tmp/removed_ros_sources.txt || true

# Install git and colcon build tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Create ROS 2 workspace
WORKDIR /root
RUN mkdir -p ros2_ws/src

# Clone repo into src directly
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/Exentr0/Test.git temp && \
    mv temp/* temp/.* ./ 2>/dev/null || true && \
    rmdir temp || true

# Build the workspace
WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# Source ROS in bash
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
