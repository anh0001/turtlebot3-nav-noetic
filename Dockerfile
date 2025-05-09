ARG BASE_IMAGE=ubuntu:20.04
FROM ${BASE_IMAGE}

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Set up locale
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Install necessary packages
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    git \
    python3-pip \
    python3-dev \
    build-essential \
    vim \
    nano \
    net-tools \
    iputils-ping \
    x11-xserver-utils \
    xterm \
    terminator \
    ssh \
    supervisor \
    xvfb \
    x11vnc

# Install noVNC
RUN mkdir -p /usr/share/novnc && \
    git clone https://github.com/novnc/noVNC.git /usr/share/novnc/noVNC && \
    git clone https://github.com/novnc/websockify /usr/share/novnc/websockify && \
    ln -s /usr/share/novnc/noVNC/vnc.html /usr/share/novnc/noVNC/index.html

# Set up ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install -y ros-noetic-desktop-full

# Install Gazebo and Turtlebot3 packages
RUN apt-get install -y \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-simulations \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-navigation \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-amcl \
    ros-noetic-gmapping \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-rviz

# Set up a lightweight desktop environment
RUN apt-get install -y \
    lxde \
    xdg-utils

# Add desktop shortcuts
RUN mkdir -p /root/Desktop
RUN echo '[Desktop Entry]\nType=Application\nName=Terminal\nExec=terminator\nIcon=terminal\nTerminal=false' > /root/Desktop/terminal.desktop && \
    chmod +x /root/Desktop/*.desktop

# Set up ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc

# Create a welcome message that displays when the container starts
RUN echo '#!/bin/bash\necho -e "\n\033[1;32mWelcome to TurtleBot3 Navigation Simulator!\033[0m\n\nAccess the simulation through your browser at: \033[1;36mhttp://localhost:6080/vnc.html\033[0m\n\n\033[1;33mCommands to run navigation:\033[0m\n\n\033[1;36m1. Launch the master file that coordinates simulation and navigation:\033[0m\n   roslaunch turtlebot3-nav-noetic turtlebot3_master.launch\n\n\033[1;36m2. Launch predefined goals with obstacle avoidance:\033[0m\n   roslaunch turtlebot3-nav-noetic turtlebot3_predefined_goals.launch\n\n\033[1;36m3. Launch patrol mode:\033[0m\n   roslaunch turtlebot3-nav-noetic turtlebot3_patrol.launch\n\n\033[1;33mIndividual script commands:\033[0m\n   rosrun turtlebot3-nav-noetic obstacle_avoidance.py\n   rosrun turtlebot3-nav-noetic patrol.py\n   rosrun turtlebot3-nav-noetic initial_pose_publisher.py\n"' > /root/welcome.sh && \
    chmod +x /root/welcome.sh && \
    echo "/root/welcome.sh" >> /root/.bashrc

# Create and set up the catkin workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Copy the project files into the workspace
COPY . /root/catkin_ws/src/turtlebot3-nav-noetic/

# Make scripts executable
RUN chmod +x /root/catkin_ws/src/turtlebot3-nav-noetic/scripts/*.py

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make"

# Set up supervisor to run Xvfb, x11vnc, and noVNC
RUN mkdir -p /var/log/supervisor
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Update supervisord.conf to run LXDE as root
RUN sed -i 's/user=rosuser/user=root/g' /etc/supervisor/conf.d/supervisord.conf

# Expose the noVNC port
EXPOSE 6080

# Start the supervisord service
CMD ["/usr/bin/supervisord"]