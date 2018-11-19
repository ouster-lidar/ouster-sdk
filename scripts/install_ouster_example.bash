#!/bin/bash
set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# get UBUNTU_CODENAME, ROS_DISTRO, REPO_DIR, CATKIN_DIR
source $SCRIPT_DIR/identify_environment.bash

if [ ! -d "$HOME/catkin_ws/src/ouster_example" ]; then
    echo "ouster_example repository not detected"
    cd "$HOME/catkin_ws/src"
    git clone https://github.com/ouster-lidar/ouster_example.git
#    cd "$HOME/catkin_ws"
#    catkin build --no-status
#    echo "Package built successfully"
else
    echo "ouster_example already installed"
fi

# Building the Sample Client
echo "building the sample client"
cd $HOME/catkin_ws/src/ouster_example/ouster_client
mkdir build
cd build
cmake ..
make
echo "sample client built successfully"

# Building the Visualizer
echo "building the visualizer"
cd $HOME/catkin_ws/src/ouster_example/ouster_viz
mkdir build
cd build
cmake ..
make
echo "visualizer built successfully"

#Building the Sample ROS Node
echo "building the sample ROS node"
cd $HOME/catkin_ws
catkin_make
echo "built the sample ROS node"

# Source the workspace
source $HOME/catkin_ws/devel/setup.bash

# get example data
echo "downloading example data"
cd ~
mkdir bags
cd bags
wget -nv https://data.ouster.io/sample-data-2018-08-29/2018-08-29-16-46-17_0.bag
echo "download complete"

