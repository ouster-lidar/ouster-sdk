# OS1 ROS Node

## Building the ROS Node
* Supports Ubuntu 16.04 with ROS Kinetic (for ouster_ros)
* ROS installation instructions can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Additionally requires ros-kinetic-pcl-ros and, optionally, ros-kinetic-rviz for visualization
* Be sure to source the ROS setup script before building. For example:`source /opt/ros/kinetic/setup.bash`
* Both packages can be built by catkin by moving them into a catkin workspace
* Build with `mkdir myworkspace && cd myworkspace && ln -s /path/to/ouster_ros ./src && catkin_make`

## Running the ROS Node
* Set up the ROS environment with `source /path/to/myworkspace/devel/setup.bash` in a new terminal for each command below
* For use with a running sensor:
  - To publish OS1 data as ROS topic `roslaunch ouster_ros os1.launch os1_hostname:=<os1_hostname> os1_udp_dest:=<udp_data_dest>` where `<os1_hostname>` can be the hostname or IP of the OS1 device and `<udp_data_dest>` is the IP to which the sensor should send data to (i.e., your computer's IP address on the interface connected to the OS1)
  - To record raw sensor output, run `rosbag record /os1_node/imu_packets /os1_node/lidar_packets` in another terminal
  - *Note: `os1_node/lidar_packets` and `os1_node/imu_packets` are the "raw data" topics, while `os1_node/points` is the ROS compatible XYZ topic and `os1_node/imu` is the ROS compatible IMU topic*
  - To visualize output, run `rviz -d /path/to/ouster_ros/rviz/viz.rviz` in another terminal
* For use with recorded sensor data:
  - To replay raw sensor output, run `roslaunch ouster_ros os1.launch replay:=true`
  - In a second terminal, run `rosbag play --clock <bagfile>`
  - To visualize output, run `rviz -d /path/to/ouster_ros/rviz/viz.rviz` in another terminal
* Sample raw sensor output is available [here](https://data.ouster.io/sample-data-2018-08-29)

## OS1 address configuration
* By default, the OS1 uses DHCP to obtain its IP address (the `<os1_hostname>` parameter above). The **"OS-1-64/16 High Resolution Imaging LIDAR: Software User Guide"** by  Ouster Inc. recommends using `dnsmasq`. 
* Setting IP address dynamically (default)
  - Install `dnsmasq` (when required), as `sudo apt install dnsmasq dnsmasq-utils`
  - Identify the interface name the OS1 is attached to. If this interface is not configured yet, setup its IP address and network mask. Use the NetworkManager and use "manual" in IPv4 settings, or use the `ifconfig` command (ex., `ifconfig <INTERFACENAME> 192.168.2.1 netmask 255.255.255.0 up`)
  - Modify the configuration file `/etc/dnsmasq.conf`, define/modify the `interface=` line with the interface name above. Also define the `dhcp-range=` value according to your interface (ex., `dhcp-range=192.168.2.50,192.168.2.150,12h`, in accordance to the `ifconfig` example above)
  - Start (stop) `dnsmasq` system service (`sudo systemctl stop dnsmasq` to stop, `sudo systemctl start dnsmasq` to start)
  - Identify the address assigned to the OS1 using `journalctl -fu dnsmasq`. Example:
```
   Oct 23 09:45:56 <HOSTNAME> dnsmasq-dhcp[30010]: DHCPREQUEST(<INTERFACENAME>) 192.168.2.*** bc:0f:**:**:**:**
   Oct 23 09:45:56 <HOSTNAME> dnsmasq-dhcp[30010]: DHCPACK(<INTERFACENAME>) 192.168.2.*** bc:0f:**:**:**:** os1-XXXXXXXXXXXX
```
where `<HOSTNAME>` is your computer's assigned hostname, `<INTERFACENAME>` is the interface configured above, `192.168.2.***` is the IP address assigned to the sensor, `bc:0f:**:**:**:**` is the OS1 MAC address and `os1-XXXXXXXXXXXX` is the sensor's hostname (`XXXXXXXXXXXX` is replaced by the sensor's serial number). *Note: instead of "\*" an actual number will be shown.*

  - You can set `<os1_hostname>` as either the assigned IP address (ex., `192.168.2.***`) or the sensor's hostname (ex., `os1-XXXXXXXXXXXX`).

## OS1 mode configuration
* Setting the LiDAR horizontal resolution and scan rate
  - Please identify the current sensor IP address or hostname using the steps above
  - The supported values for `lidar_mode` are: 512x10, 1024x10, 2048x10, 512x20, 1024x20. The first value is horizontal resolution and the second is scan rate.
  - Connect to the sensor and reconfigure the `lidar_mode` parameter, storing the modification internally so to use it at next boot. You can skip the `write_config_txt` command to use default value at next boot. `os1-XXXXXXXXXXXX` is the sensor's hostname (`XXXXXXXXXXXX` is replaced by the sensor's serial number).
```bash
  nc os1-XXXXXXXXXXXX 7501
  get_config_param active lidar_mode
  set_config_param lidar_mode 2048x10
  write_config_txt
  reinitialize
```

##Todo
- [x] Velodyne compatibility mode.
- [x] Configure active sensor parameters from driver code, in particular `lidar_mode`.
- [ ] Fix `replay` option currently not working.
- [ ] Verify the sensor operation information and attempt software reset on error condition, and current parameter values to avoid unnecessary rewrites/reinitialization.
