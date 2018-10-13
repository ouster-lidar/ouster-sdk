# OS1 Example Client and ROS Node

## Contents
* `ouster_client/` contains a simple C++ client for the OS1 sensor
* `ouster_ros/` contains an example ROS node for publishing point cloud messages
* `ouster_viz/` contains a  visualizer for an OS1 sensor

## Building the Sample Client
* The sample client requires a compiler supporting C++11 or newer and CMake
* Build with `cd /path/to/ouster_example/ouster_client && mkdir build && cd build && cmake .. && make`

## Running the Sample Client
* The sample client includes a small driver program that just prints some data to the terminal
* Make sure the OS1 is connected to the network and has obtained a dhcp lease. See accompanying documentation for more details
* You should see a binary called `ouster_client_example` in your build directory on success
* Run `ouster_client_example <os1_hostname> <udp_data_dest>` where `<os1_hostname>` is the hostname or IP address of the OS1 sensor, and `<udp_data_dest>` is the IP to which the sensor should send lidar data

## Building the visualizer
* The visualizer is not built using ROS
* Build with `cd /path/to/ouster_example/ouster_viz && mkdir build && cd build && cmake .. && make`

## Running the visualizer (does not require ROS)
* The example visualiser can be used to display point clouds and range/intensity/noise images for the OS1 sensor
* You should see a binary called "viz" in your build directory on success
* Run `./viz <os1_hostname> <udp_data_dest>`, passing the same arguments as for `ouster_client_example`

## Building the Sample ROS Node
* Supports Ubuntu 16.04 with ROS Kinetic (for ouster_ros)
* ROS installation instructions can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Additionally requires ros-kinetic-pcl-ros and, optionally, ros-kinetic-rviz for visualization
* Be sure to source the ROS setup script before building. For example:`source /opt/ros/kinetic/setup.bash`
* Both packages can be built by catkin by moving them into a catkin workspace
* Build with `mkdir myworkspace && cd myworkspace && ln -s /path/to/ouster_example ./src && catkin_make`

## Running the Sample ROS Node
* Set up the ROS environment with `source /path/to/myworkspace/devel/setup.bash` in a new terminal for each command below
* For use with a running sensor:
  - To publish OS1 data as ROS topic `roslaunch ouster_ros os1.launch os1_hostname:=<os1_hostname> os1_udp_dest:=<udp_data_dest>` where `<os1_hostname>` can be the hostname or IP of the OS1 device and `<udp_data_dest>` is the IP to which the sensor should send data
  - To record raw sensor output, run `rosbag record /os1_node/imu_packets /os1_node/lidar_packets` in another terminal
  - Note: os1_node/lidar_packets and os1_node/imu_packets are the "raw data" topics, while os1_node/points is the ROS compatible XYZ topic and os1_node/imu is the ROS compatible IMU topic
  - To visualize output, run `rviz -d /path/to/ouster_ros/viz.rviz` in another terminal
* For use with recorded sensor data:
  - To replay raw sensor output, run `roslaunch ouster_ros os1.launch replay:=true`
  - In a second terminal, run `rosbag play --clock <bagfile>`
  - To visualize output, run `rviz -d /path/to/ouster_ros/viz.rviz` in another terminal
* Sample raw sensor output is available [here](https://data.ouster.io/sample-data-2018-08-29)
