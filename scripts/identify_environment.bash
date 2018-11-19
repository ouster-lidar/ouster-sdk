#!/bin/bash
set -e
export UBUNTU_CODENAME=$(lsb_release -s -c)
case $UBUNTU_CODENAME in
  trusty)
    export ROS_DISTRO=indigo;;
  xenial)
    export ROS_DISTRO=kinetic;;
  bionic)
    export ROS_DISTRO=melodic;;
  *)
    echo "Unsupported version of Ubuntu detected. Only trusty (14.04.*), xenial (16.04.*), and bionic (18.04.*) are currently supported."
    exit 1
esac
export REPO_DIR=$(dirname "$SCRIPT_DIR")
export CATKIN_DIR="$HOME/catkin_ws"
