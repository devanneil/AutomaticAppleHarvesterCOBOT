FROM ros-humble-duco-base

RUN mkdir -p /home/$USERNAME/ws/src

SHELL ["/bin/bash", "-c"]

WORKDIR /home/$USERNAME/ws
# Copy dependency files only (for cache efficiency) 
COPY apt-packages.txt /tmp/apt-packages.txt 
COPY pip-requirements.txt /tmp/requirements.txt
# Copy entrypoint file 
COPY entrypoint.sh /tmp/entrypoint.sh 
RUN sudo chmod -R +777 /tmp 
# Install apt dependencies in ONE layer 
RUN --mount=type=cache,target=/var/cache/apt \
 sudo apt-get update \
  && sudo apt-get install -y --no-install-recommends \
   $(tr '\n' ' ' < /tmp/apt-packages.txt) \
    && sudo rm -rf /var/lib/apt/lists/* 
# Install pip dependencies 
RUN --mount=type=cache,target=/root/.cache/pip \
python3 -m pip install --upgrade pip \
  && pip3 install --no-cache-dir -r /tmp/requirements.txt

RUN echo "export DISPLAY=host.docker.internal:0" >> /home/$USERNAME/.bashrc \
 && echo "" >> /home/$USERNAME/.bashrc \
 && echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
 && echo "if [ -f /home/ws/install/setup.bash ]; then" >> /home/$USERNAME/.bashrc \
 && echo "    source /home/ws/install/setup.bash" >> /home/$USERNAME/.bashrc \
 && echo "fi" >> /home/$USERNAME/.bashrc

USER $USERNAME

RUN sudo rosdep fix-permissions

# Add ros-dev to video group 
RUN sudo groupadd -f video && sudo usermod -aG video $USERNAME 

# Add ros-dev to dialout group 
RUN sudo groupadd -f dialout && sudo usermod -aG dialout $USERNAME

#=================================Vsense ROS2 Setup=========================
RUN mkdir -p /home/$USERNAME/vsense_ros2 \
 && cd /home/$USERNAME/vsense_ros2 \
 && git clone https://github.com/ScepterSW/ScepterSDK ScepterSDK \
 && cd /home/$USERNAME/vsense_ros2/ScepterSDK/3rd-PartyPlugin/ROS2/src/ScepterROS_MultiCameras \
 && python3 install.py Ubuntu22.04 \
 && cd ../../ && source /opt/ros/humble/setup.sh \
 && colcon build --merge-install --install-base install --packages-select ScepterROS_MultiCameras \
 && echo "source /home/ros-dev/vsense_ros2/ScepterSDK/3rd-PartyPlugin/ROS2/install/setup.bash" >> ~/.bashrc

#=================================Vsense ROS2 Setup=========================

SHELL ["/bin/bash", "-c"] 
ENV SHELL=/bin/bash 
ENTRYPOINT [ "/tmp/entrypoint.sh" ] 
CMD ["/bin/bash"]