# Example Sensor Client

## Contents
* `ouster_client/` contains a simple C++ client application that prints lidar
  data to the terminal
* can be built both with and without ROS. See the instructions in
  [ouster_ros/](../ouster_ros/README.md) for building in a ROS environment

## Building the Sample Client
* The sample client requires a compiler supporting C++11 or newer and CMake
* Build with `cd /path/to/ouster_example/ouster_client && mkdir build && cd
  build && cmake .. && make` where `/path/to/ouster_example` is where you've
  cloned the repository

## Running the Sample Client
* Make sure the sensor is connected to the network and has obtained a DHCP
  lease. See section 3.1 in the accompanying
  [software user guide](https://www.ouster.io/downloads) for more details
* An executable called `ouster_client_example` is generated in the build
  directory on success
* Run `./ouster_client_example <sensor_hostname> <udp_data_dest_ip>` where
  `<sensor_hostname>` is the hostname or IP address of the sensor, and
  `<udp_data_dest_ip>` is the IP to which the sensor should send lidar data
