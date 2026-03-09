FROM osrf/ros:humble-desktop
ARG USERNAME=ros-dev
ENV USERNAME=$USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive
#-------------------DUCO Setup---------------------------------------
# Set software sources
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/ $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# Add key
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654
RUN apt update
#====================================================================

# Install apt dependencies in ONE layer
RUN --mount=type=cache,target=/var/cache/apt \
 apt-get update && \
 apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-test-msgs \
    ros-humble-filters \
    ros-humble-control-toolbox \
    ros-humble-ackermann-msgs \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro  \
    python3-pip \
    python3-vcstool \
    wget
# Install pip dependencies
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install --upgrade pip colcon-common-extensions

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************
# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

# update ROS dependencies (must be below the USER command)
# this is a way to continue if the command fails (the `|| true`)
RUN sudo rosdep init; rosdep update

# Add ros-dev to video group
RUN sudo groupadd -f video && sudo usermod -aG video $USERNAME

# Add ros-dev to dialout group
RUN sudo groupadd -f dialout && sudo usermod -aG dialout $USERNAME

#-------------------DUCO Setup---------------------------------------
RUN mkdir -p /home/$USERNAME/ros2_control_ws/src \
 && cd /home/$USERNAME/ros2_control_ws \
 && wget https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.humble.repos \
 && vcs import src < ros_controls.humble.repos

# Copy DUCO source SDK into /home/$username/ros2_control_ws/src here

RUN rosdep init || true \
 && rosdep update \
 && cd /home/$USERNAME/ros2_control_ws \
 && rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/humble/setup.sh \
 && cd /home/$USERNAME/ros2_control_ws \
 && colcon build

RUN echo "source /home/$USERNAME/ros2_control_ws/install/setup.bash" >> /home/$USERNAME/.bashrc
#===========================================================