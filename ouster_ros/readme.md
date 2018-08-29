#README for OS1 Example ROS Node

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
  - To publish OS1 data as ROS topic `roslaunch ouster_ros os1.launch os1_hostname:=<os1_hostname> os1_udp_dest=<udp_data_dest_ip>` where `<os1_hostname>` can be the hostname or IP of the OS1 device and `<udp_data_dest_ip>` is the IP to which the sensor should send data
  - To record raw sensor output, run `rosbag record /os1_node/imu_packets /os1_node/lidar_packets` in another terminal
  - To visualize output, run `rviz -d /path/to/ouster_ros/viz.rviz` in another terminal
* For use with recorded sensor data:
  - To replay raw sensor output, run `roslaunch ouster_ros os1.launch replay:=true`
  - In a second terminal, run `rosbag play --clock <bagfile>`
  - To visualize output, run `rviz -d /path/to/ouster_ros/viz.rviz` in another terminal
* Sample raw sensor output is available [here](https://data.ouster.io/ouster-os1-100sec.bag)
