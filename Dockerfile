FROM ros:jazzy

# Set environment variables
ENV ROS_DISTRO=jazzy
ENV PYTHONPATH=/opt/ros/jazzy/lib/python3.10/site-packages:$PYTHONPATH

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-venv \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /ros2_ws/src/cropion_rover

# Copy project files
COPY . .

# Install Python dependencies
RUN python3 -m venv /opt/venv
# Activate virtual environment and install requirements
RUN . /opt/venv/bin/activate && \
    pip3 install --upgrade pip && \
    pip3 install -r requirements.txt

# Build ROS2 workspace
WORKDIR /ros2_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

# Source workspace and venv in bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /opt/venv/bin/activate" >> /root/.bashrc

# Entry point script
# Modify docker-entrypoint.sh to activate venv
COPY docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh && \
    sed -i '2i source /opt/venv/bin/activate' /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["ros2", "launch", "cropion_rover", "rover.launch.py"] 