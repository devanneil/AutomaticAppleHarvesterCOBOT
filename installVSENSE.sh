#=================================Vsense ROS2 Setup=========================
mkdir -p /home/$USERNAME/vsense_ros2 
cd /home/$USERNAME/vsense_ros2 
git clone https://github.com/ScepterSW/ScepterSDK ScepterSDK 
cd /home/$USERNAME/vsense_ros2/ScepterSDK/3rd-PartyPlugin/ROS2/src/ScepterROS_MultiCameras 
python3 install.py Ubuntu22.04 
cd ../../  
source /opt/ros/humble/setup.sh 
colcon build --merge-install --install-base install --packages-select ScepterROS_MultiCameras 
echo "source /home/ros-dev/vsense_ros2/ScepterSDK/3rd-PartyPlugin/ROS2/install/setup.bash" >> ~/.bashrc
#=================================Vsense ROS2 Setup=========================