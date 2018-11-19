#!/bin/bash
set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# get UBUNTU_CODENAME, ROS_DISTRO, REPO_DIR, CATKIN_DIR
source $SCRIPT_DIR/identify_environment.bash

# Download Google Cartographer
echo "downloading Google Cartographer..."
sudo apt-get install ros-melodic-cartographer ros-melodic-cartographer-ros ros-melodic-cartographer-ros-msgs ros-melodic-cartographer-rviz -y

# Copy Configuration files to the default location (/opt/ros/melodic/share/cartographer_ros)
echo "Copying configuration files..."
sudo cp $HOME/catkin_ws/src/ouster_example/cartographer_ros/configuration_files/* /opt/ros/melodic/share/cartographer_ros/configuration_files/

sudo cp $HOME/catkin_ws/src/ouster_example/cartographer_ros/launch/* /opt/ros/melodic/share/cartographer_ros/launch/

sudo cp $HOME/catkin_ws/src/ouster_example/cartographer_ros/urdf/* /opt/ros/melodic/share/cartographer_ros/urdf/

#sudo mv /bags/* ~/bags/
#sudo rm -R /bags/
 

