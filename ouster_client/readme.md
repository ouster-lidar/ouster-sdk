README for OS1 Example Client

## Contents
* `ouster_client/` contains a simple C++ client for the OS1 sensor

## Building the Sample Client
* The sample client requires a compiler supporting C++11 or newer and CMake
* Build with `cd /path/to/ouster_example/ouster_client && mkdir build && cd build && cmake .. && make`

## Running the Sample Client
* The sample client includes a small driver program that just prints some data to the terminal
* Make sure the OS1 is connected to the network and has obtained a dhcp lease. See accompanying documentation for more details
* You should see a binary called `ouster_client_example` in your build directory on success
* Run `ouster_client_example <os1_hostname> <udp_data_dest_ip>` where `<os1_hostname>` is the hostname or IP address of the OS1 sensor, and `<udp_data_dest_ip>` is the IP to which the sensor should send lidar data

