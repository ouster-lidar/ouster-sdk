# Example Visualizer

## Contents
* `ouster_viz/` contains a basic visualizer that can be used to
  display point clouds and range/intensity/ambient images
* Can be built both with and without ROS. See the instructions in
  [ouster_ros](../ouster_ros/README.md) for building in a ROS environment

## Operating System Support
The visualizer is cross-platform and works on Linux, Mac, and Windows. The current supported
platforms are:

* Ubuntu 16.04
* Ubuntu 18.04
* Arch Linux

Preliminary compatibility has been added for:

* Windows 10
* macOS 10.15

though these are not officially supported and not yet guaranteed to build and run correctly.

# Build Dependencies
* The sample visualizer requires a compiler supporting C++11 or newer
  and CMake 3.1 or newer
* Requires GLFW3, GLEW and Eigen3 libraries
* Using Ubuntu: `sudo apt-get install libglfw3-dev libglew-dev libeigen3-dev libjsoncpp-dev libtclap-dev`
* Using Arch Linux: `sudo pacman -S cmake glfw glew eigen jsoncpp tclap`
* Using MacOS: install XCode and [Brew](https://brew.sh/) and then `brew install cmake pkg-config glfw glew eigen jsoncpp tclap`
* Using Windows: install Visual Studio and [vcpkg](https://github.com/microsoft/vcpkg) and then `vcpkg install glfw3 glew tclap jsoncpp eigen3`

# Building the Visualizer
Before building, make sure all the build dependencies are installed.

In the following instruction steps, `/path/to/ouster_example` is where you've cloned the repository

## Linux
Run the following commmands:

```
export CMAKE_PREFIX_PATH=/path/to/ouster_example
cd /path/to/ouster_example/ouster_viz
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

## macOS
Run the following commands

```
export CMAKE_PREFIX_PATH=/path/to/ouster_example
cd /path/to/ouster_example/ouster_viz
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF ..
make
```

## Running the Visualizer
* An executable called `simple_viz` is generated in the build directory
* Note: if compiling in an environment with ROS, the location of the
  executable will be different
* To run: `./simple_viz <flags> <sensor_hostname> <udp_data_dest_ip>`
* For help, run `./simple_viz -h`

## Command Line Arguments
* `<sensor_hostname>` the hostname or IP address of the sensor
* `<udp_data_dest_ip>` the IP to which the sensor should send data
* `-m <512x10 | 512x20 | 1024x10 | 1024x20 | 2048x10>` flag to set the lidar
  mode (horizontal resolution x rotation rate). Defaults to 1024x10.

## Key bindings
| key | what it does |
| ----| ------------ |
| `p` | Increase point size |
| `o` | Decrease point size |
| `m` | Cycle point cloud coloring by intensity, noise, reflectivity, range |
| `v` | Toggle color cycling in range image |
| `n` | Toggle display ambient image from the sensor |
| `r` | Toggle auto-rotating |
| `shift + r` | Reset camera |
| `e` | Change range and intensity image size|
| `;` | Increase spacing in range markers |
| `'` | Decrease spacing in range markers |
| `r` | Toggle auto rotate |
| `w` | Camera pitch up |
| `s` | Camera pitch down |
| `a` | Camera yaw left |
| `d` | Camera yaw right |
| `1` | Toggle point cloud visibility |
| `0` | Toggle orthographic camera |
| `=` | Zoom in |
| `-` | Zoom out |

## Mouse control
* Click and drag rotates the view
* Middle click and drag moves the view
* Scroll adjusts how far away the camera is from the vehicle
