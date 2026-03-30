sudo apt-get update 
sudo apt-get install -y \
    python3-colcon-common-extensions \
    python3-colcon-override-check \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-test-msgs \
    ros-humble-filters \
    ros-humble-control-toolbox \
    ros-humble-ackermann-msgs \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro  \
    ros-humble-warehouse-ros \
    libmongoc-dev \
    libbson-dev \
    python3-pip \
    python3-vcstool \
    wget
export USERNAME=rnall
mkdir /home/${USERNAME}/duco_ros_tmp
cp -r duco_ros_sdk/duco_ros2_ws /home/${USERNAME}/duco_ros_tmp
#-------------------DUCO Setup---------------------------------------
# Build and compile ros2-warehouse-mongo
# Pull ros2 control source
mkdir -p /home/$USERNAME/ros2_control_ws/src 
cd /home/$USERNAME/ros2_control_ws/src 
git clone https://github.com/ros-planning/warehouse_ros_mongo.git -b ros2 
rosdep install --from-paths . --ignore-src -y 
cd /home/$USERNAME/ros2_control_ws 
. /opt/ros/humble/setup.sh 
#  colcon build --symlink-install 
wget https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.humble.repos 
vcs import src < ros_controls.humble.repos

# Copy DUCO source SDK into /home/$username/ros2_control_ws/src here
cp -r /home/${USERNAME}/duco_ros_tmp /home/${USERNAME}/ros2_control_ws/src/
rm -rf duco_ros_tmp

rosdep init || true 
rosdep update 
sudo apt-get update 
cd /home/$USERNAME/ros2_control_ws 
rosdep install --from-paths src --ignore-src -r -y

source /opt/ros/humble/setup.sh 
cd /home/$USERNAME/ros2_control_ws 
colcon build --symlink-install --packages-select control_toolbox 
source install/setup.bash 
colcon build --symlink-install --packages-up-to hardware_interface 

cd /home/$USERNAME/ros2_control_ws 
unset AMENT_PREFIX_PATH 
unset CMAKE_PREFIX_PATH 
source /opt/ros/humble/setup.bash 
export ackermann_msgs_DIR=~/ackermann_ws/install/ackermann_msgs/share/ackermann_msgs/cmake 
sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_control_ws 
rm -rf build install log 
colcon build --merge-install --install-base install --packages-select duco_msg 
source /opt/ros/humble/setup.bash 
source install/setup.bash 
colcon build --merge-install --install-base install --allow-overriding control_msgs ackermann_msgs

echo "source /home/$USERNAME/ros2_control_ws/install/setup.bash" >> /home/$USERNAME/.bashrc
#===========================================================