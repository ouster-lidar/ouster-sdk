# README

## Contents
* 'ouster_viz/' contains a basic visualizer that can be used to display point clouds and range/intensity/noise images
*  This visualizer does not depend on ROS

## Operating System Support
* The visualizer has been tested on: Ubuntu 16.04, 17.1, and 18.04, as well as Fedora 28, and Raspian Jessie
* Windows and Mac support is in progress

## Build Dependencies
* The sample visualizer requires a compiler supporting C++11 or newer and CMake 3.1 or newer
* Requires VTK6 and Eigen3 libraries
* Using Ubuntu: sudo apt-get install libvtk6-dev libeigen3-dev eigen3-devel.noarch
* Using Fedora: sudo apt-get update yum install vtk-devel.x86_64

## Building the Visualizer:
* Build with 'cd /path/to/ouster_example/ouster_viz && mkdir build && cd build && cmake .. && make'

## Running the Visualizer
* An executable called "viz" is generated in the build directory
* Note: if compiling with catkin the location of the executable will be different
* To run: `./viz <os1_hostname> <udp_data_dest_ip>` where `<os1_hostname>` is the hostname or IP address of the OS1 sensor, and `<udp_data_dest_ip>` is the IP to which the sensor should send data
* For help, run ./viz -h

## Command Line Arguments
* -m  <512x10 | 512x20 | 1024x10 | 1024x20 | 2048x10> Set the lidar mode of the visualizer - needs to be in the same setting as the sensor is configured. Default: 1024x10
* -r <float>  Set the multiplier for the range image visualization scaling factor
* -i <float> Set the multiplier for the intensity image visualization scaling factor
* -n <float> Set the multiplier for the noise image visualization scaling factor

## Key bindings

| key | what it does |
| ---| ---------|
| `o` | Increase point size |
| `p` | Decrease point size |
| `i` | Colour 3D points by intensity |
| `z` | Colour 3D points by z-height |
| `shift z` | Colour 3D points by z-height plus intensity |
| `r` | Colour 3D points by range |
| `c` | Cycle colour for accumulated 3D points and range image |
| `shift c` | Cycle colour for latest 3D points |
| `v` | Toggle colour cycling in range image |
| `n` | Display noise image from the sensor|
| `a` | Rotate camera to show 3D points in top down view |
| `f` | Rotate camera to show 3D points in front view |
| `left` | Rotate camera left |
| `right` | Rotate camera right |
| `up` | Rotate camera up |
| `down` | Rotate camera down |
| `+` | Move camera closer |
| `-` | Move camera farther |
| `0` (zero) | Toggle parallel projection |
| `d` | Decrease fraction of the window height for displaying range and intensity image |

## Mouse control

* Click and drag rotates the view
* Middle click and drag pans the view (will be reset the next time you adjust the view)
* Scroll adjusts how far away the camera is from the vehicle

## Modes

* **Cycle range image colour.** If true, colour is repeated every few meters. If false, colour is monotone with respect to range. Disabled by default, press `v` to toggle
* **Parallel projection.** If true, renders 3D point cloud without perspective distortion. If false, renders 3D point cloud as though with a wide angle lens. Disabled by default, press `0` (zero) to toggle
* **Colour mode.** Possible values: z-height, intensity, range, and z-height plus intensity. Z-height plus intensity by default. Press `z`, `i`, `r`, and `shift z` to select

# Troubleshooting

## No data being displayed

**Cause**: `<os1_hostname>` or `<udp_data_dest_ip>` are incorrect and packets are not being sent

**Solution**: ensure that you can resolve and ping both `<os1_hostname>` and `<udp_data_dest_ip>`

## Vertical black stripes in Lidar Scan or Lidar Scan updating too slowly

**Cause**: Sensor configured with a different lidar_mode than the visualizer

**Solution**: Ensure that sensor and visualizer are in the same mode

To configure lidar_mode on the sensor using the TCP interface via the `netcat` utility:  
    `nc <os1_hostname> <TCP_port>` (example: nc 192.168.1.121 7501)  
    `set_config_param lidar_mode 1024x10`  
    `reinitialize`  
    `write_config_txt` (optionally, to persist changes)
