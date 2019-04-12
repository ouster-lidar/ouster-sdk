# OS-1 Example ROS Node

## Contents
* `ouster_ros/` contains sample code for publishing OS-1 data as standard ROS
  topics
* Tested with ROS Kinetic on Ubuntu 16.04

## Building the Sample ROS Node
* Supports Ubuntu 16.04 with ROS Kinetic (for ouster_ros)
* ROS installation instructions can be found
  [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Additionally requires `ros-kinetic-pcl-ros`, `ros-kinetic-tf2-geometry-msgs`
  and, optionally, `ros-kinetic-rviz` for visualization using ROS
* Be sure to source the ROS setup script before building. For example:`source
  /opt/ros/kinetic/setup.bash`
* Build with `mkdir -p myworkspace/src && cd myworkspace && ln -s
  /path/to/ouster_example ./src/ && catkin_make -DCMAKE_BUILD_TYPE=Release`

## Running the Sample ROS Nodes
* Make sure to set up the ROS environment with `source
  /path/to/myworkspace/devel/setup.bash` in a new terminal for each command
  below
* To publish ROS topics from a running sensor:
    - Run `roslaunch ouster_ros os1.launch os1_hostname:=<os1_hostname>
     os1_udp_dest:=<udp_data_dest_ip> lidar_mode:=<lidar_mode>` where
     `<os1_hostname>` can be the hostname or IP of the OS-1 device,
     `<udp_data_dest_ip>` is the IP to which the sensor should send data, and
     `<lidar_mode>` is one of 512x10, 512x20, 1024x10, 1024x20, or 2048x10
    - To record raw sensor output, run `rosbag record /os1_node/imu_packets
     /os1_node/lidar_packets` in another terminal
* To publish ROS topics from recorded data:
    - Run `roslaunch ouster_ros os1.launch replay:=true
      os1_hostname:=<os1_hostname>`
    - In a second terminal run `rosbag play --clock <bagfile>`
    - Note: `os1_node` reads and writes metadata to `${ROS_HOME}` to enable
      accurately replaying raw data. By default, the name of this file is based
      on the hostname of the sensor. The location of this file can be overridden
      using the `metadata:=<path_to_file>` flag
    - If a metadata file is not available, the visualizer will default to
      1024x10. This can be overridden with the `lidar_mode`
      parameter. Visualizer output will only be correct if the same `lidar_mode`
      parameter is used for both recording and replay
* To display sensor output using the provided visualizer:
    - To visualize the published OS-1 point cloud data using the provided
      visualizer, add `viz:=true` to either of the `roslaunch` commands above
    - A window should open and start displaying data after a few seconds. This
      should work with a running sensor or replayed data
    - See the [README.md](../ouster_viz/README.md) in the `ouster_viz` directory
      for details on keyboard and mouse controls
* To display sensor output using ROS tools (rviz):
    - Follow the instructions above for running the example ROS code with a
      sensor or recorded data
    - To visualize output using rviz, run `rviz -d /path/to/ouster_ros/viz.rviz`
      in another terminal
    - To view lidar intensity/noise/range images, add `image:=true` to either of
      the `roslaunch` commands above
