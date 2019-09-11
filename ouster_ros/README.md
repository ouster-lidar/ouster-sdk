# OS1 Example ROS Node

## Contents
* `ouster_ros/` contains sample code for publishing OS1 data as standard
 ROS topics

## Operating System Support
* The visualizer has been tested on [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) on Ubuntu 16.04
    * additionally requires `ros-kinetic-pcl-ros`, `ros-kinetic-tf2-geometry-msgs`
      and, optionally, `ros-kinetic-rviz` for visualization using ROS
* The visualizer has been tested on [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) on Ubuntu 18.04
    * additionally requires `ros-melodic-pcl-ros`, `ros-melodic-tf2-geometry-msgs`
      and, optionally, `ros-melodic-rviz` for visualization using ROS

## Building the Sample ROS Node
* In the following instruction steps, `/path/to/ouster_example` is where you've cloned the repository 
* Run the following command `export CMAKE_PREFIX_PATH=/path/to/ouster_example`
* Be sure to source the ROS setup script before building. For example:
`source /opt/ros/[kinetic_or_melodic]/setup.bash`
* Build with `mkdir -p myworkspace/src && cd myworkspace && ln -s
  /path/to/ouster_example ./src/ && catkin_make -DCMAKE_BUILD_TYPE=Release`

## Running the Sample ROS Nodes
* Make sure the OS1 is connected to the network and has obtained a DHCP lease. See section 3.1 in the accompanying [software user guide](https://www.ouster.io/downloads) for more details
* In each new terminal for each command below:
    - Make sure to source ROS environment with `source
  /path/to/myworkspace/devel/setup.bash` where `/path/to/myworkspace` is where the path to the workspace directory that was created 
  when building the ROS nodes
* To publish ROS topics from a running sensor from within the `ouster_ros` directory:
    - Run `roslaunch os1.launch os1_hostname:=<os1_hostname>
     os1_udp_dest:=<udp_data_dest_ip> lidar_mode:=<lidar_mode> viz:=<viz>`where:
        - `<os1_hostname>` can be the hostname (os1-991xxxxxxxxx) or IP of the OS1
        - `<udp_data_dest_ip>` is the IP to which the sensor should send data
        - `<lidar_mode>` is one of `512x10`, `512x20`, `1024x10`, `1024x20`, or `2048x10`
        - `<viz>` is either `true` or `false`. If true, a window should open and start 
          displaying data after a few seconds
* To record raw sensor output
    - In another terminal instance, run `rosbag record /os1_node/imu_packets
     /os1_node/lidar_packets`
    - This will save a .bag file of recorded data in that directory
* To publish ROS topics from recorded data from withint the `ouster_ros` directory:
    - Run `roslaunch os1.launch replay:=true
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
* To display sensor output using ROS tools (rviz):
    - Follow the instructions above for running the example ROS code with a
      sensor or recorded data
    - To visualize output using rviz, run `rviz -d /path/to/ouster_ros/viz.rviz`
      in another terminal
    - To view lidar intensity/noise/range images, add `image:=true` to either of
      the `roslaunch` commands above

## Key bindings
| key | what it does |
| ----| ------------ |
| `o` | Increase point size |
| `p` | Decrease point size |
| `m` | Cycle point cloud coloring by z-height / intensity / z-height plus intensity / range |
| `c` | Cycle color scheme for range image |
| `shift c` | Cycle color scheme for point cloud |
| `v` | Toggle color cycling in range image |
| `n` | Display ambient image from the sensor|
| `r` | Reset camera position
| `0` (zero) | Toggle parallel projection and reset camera |
| `d` | Cycle through fraction of the window height used for displaying range and intensity image

## Mouse control
* Click and drag rotates the view
* Middle click and drag pans the view
* Scroll adjusts how far away the camera is from the vehicle 
