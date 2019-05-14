# OS1 Example Visualizer

## Contents
* `ouster_viz/` contains a basic visualizer that can be used to
  display point clouds and range/intensity/ambient images
* Can be built both with and without ROS. See the instructions in
  [ouster_ros](../ouster_ros/README.md) for building in a ROS environment

## Operating System Support
* The visualizer has been tested on: Ubuntu 16.04, 17.1, and 18.04, as
  well as Fedora 28, and Raspbian Jessie
* Windows and Mac support is in progress

## Build Dependencies
* The sample visualizer requires a compiler supporting C++11 or newer
  and CMake 3.1 or newer
* Requires VTK6 and Eigen3 libraries
* Using Ubuntu: sudo apt-get install libvtk6-dev libeigen3-dev
* Using Fedora: sudo yum install vtk-devel.x86_64 eigen3-devel.noarch

## Building the Visualizer:
* In the following instruction steps, `/path/to/ouster_example` is where you've cloned the repository
* Run the following command `export CMAKE_PREFIX_PATH=/path/to/ouster_example`
* Build with `cd /path/to/ouster_example/ouster_viz && mkdir build &&
  cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make`

## Running the Visualizer
* An executable called `viz` is generated in the build directory
* Note: if compiling in an environment with ROS, the location of the
  executable will be different
* To run: `./viz <flags> <os1_hostname> <udp_data_dest_ip>`
* For help, run `./viz -h`

## Command Line Arguments
* `<os1_hostname>` the hostname or IP address of the OS1 sensor
* `<udp_data_dest_ip>` the IP to which the sensor should send data
* `-m <512x10 | 512x20 | 1024x10 | 1024x20 | 2048x10>` flag to set the lidar
  mode (horizontal resolution x rotation rate). Defaults to 1024x10.

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
| `d` | Cycle through fraction of the window height used for displaying range and intensity image |

## Mouse control
* Click and drag rotates the view
* Middle click and drag pans the view
* Scroll adjusts how far away the camera is from the vehicle
