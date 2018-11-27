#!/bin/bash
set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# get UBUNTU_CODENAME, ROS_DISTRO, REPO_DIR, CATKIN_DIR
source $SCRIPT_DIR/identify_environment.bash

export DEBIAN_FRONTEND=noninteractive

sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu $UBUNTU_CODENAME main\" > /etc/apt/sources.list.d/ros-latest.list"
wget -qO - http://packages.ros.org/ros.key | sudo apt-key add -

echo "Updating package lists ..."
sudo apt-get -qq update

echo "Installing additional ROS $ROS_DISTRO packages ..."

sudo apt-get -qq install ros-$ROS_DISTRO-rviz ros-$ROS_DISTRO-gazebo-* ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-controller-manager ros-$ROS_DISTRO-joint-state-controller ros-$ROS_DISTRO-ros-control > /dev/null

source /opt/ros/$ROS_DISTRO/setup.bash

