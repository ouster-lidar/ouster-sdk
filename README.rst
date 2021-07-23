===================
Ouster Example Code
===================

:Description: Sample code provided for working with Ouster sensors

.. contents:: Contents:
   :local:


Summary
=======

To get started building the client and visualizer libraries, see the `Sample Client and Visualizer`_
section below. For instructions on ROS, start with the `Example ROS Code`_ section. Python SDK users
should proceed straight to our `python SDK homepage <python/>`_.

This repository contains sample code for connecting to and configuring ouster sensors, reading and
visualizing data, and interfacing with ROS.

* `ouster_client <ouster_client/>`_ contains an example C++ client for ouster sensors
* `ouster_viz <ouster_viz/>`_ contains a basic point cloud visualizer
* `ouster_ros <ouster_ros/>`_ contains example ROS nodes for publishing point cloud messages
* `python <python/>`_ contains the code for the ouster sensor python SDK


Sample Client and Visualizer
============================

Building the example code requires a compiler supporting C++11 and CMake 3.1 or newer and the tclap,
jsoncpp, and Eigen3 libraries with headers installed on the system. The sample visualizer also
requires the GLFW3 and GLEW libraries.

Building on Linux / macOS
-------------------------

To install build dependencies on Ubuntu, run::

    sudo apt install build-essential cmake libglfw3-dev libglew-dev libeigen3-dev \
         libjsoncpp-dev libtclap-dev

On macOS, install XCode and `homebrew <https://brew.sh>`_ and run::

    brew install cmake pkg-config glfw glew eigen jsoncpp tclap

To build run the following commands::

    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release <path to ouster_example>
    make

where ``<path to ouster_example>`` is the location of the ``ouster_example`` source directory. The
CMake build script supports several optional flags::

    -DBUILD_VIZ=OFF                      Do not build the sample visualizer
    -DBUILD_PCAP=ON                      Build pcap tools. Requres libpcap and libtins dev packages
    -DBUILD_SHARED_LIBS                  Build shared libraries (.dylib or .so)
    -DCMAKE_POSITION_INDEPENDENT_CODE    Standard flag for position independent code

Building on Windows
-------------------

The example code can be built on Windows 10 with Visual Studio 2019 using CMake support and vcpkg
for dependencies. Follow the official documentation to set up your build environment:

* `Visual Studio <https://visualstudio.microsoft.com/downloads/>`_
* `Visual Studio CMake Support
  <https://docs.microsoft.com/en-us/cpp/build/cmake-projects-in-visual-studio?view=vs-2019>`_
* `Visual Studio CPP Support
  <https://docs.microsoft.com/en-us/cpp/build/vscpp-step-0-installation?view=vs-2019>`_
* `Vcpkg, at tag "2020.11-1" installed and integrated with Visual Studio
  <https://docs.microsoft.com/en-us/cpp/build/vcpkg?view=msvc-160#installation>`_

**Note** You'll need to run ``git checkout 2020.11-1`` in the vcpkg directory before bootstrapping
to use the correct versions of the dependencies. Building may fail unexpectedly if you skip this
step.

Don't forget to integrate vcpkg with Visual Studio after bootstrapping::

    .\vcpkg.exe integrate install

You should be able to install dependencies with::

    .\vcpkg.exe install --triplet x64-windows glfw3 glew tclap jsoncpp eigen3

After these steps are complete, you should be able to open, build and run the ``ouster_example``
project using Visual Studio:

1. Start Visual Studio.
2. When the prompt opens asking you what type of project to open click **Open a local folder** and
   navigate to the ``ouster_example`` source directory.
3. After opening the project for the first time, wait for CMake configuration to complete.
4. Make sure Visual Studio is `building in release mode`_. You may experience performance issues and
   missing data in the visualizer otherwise.
5. In the menu bar at the top of the screen, select **Build > Build All**.
6. To use the resulting binaries, go to **View > Terminal** and run, for example::

    .\out\build\x64-Release\ouster_client\ouster_client_example.exe -h

.. _building in release mode: https://docs.microsoft.com/en-us/visualstudio/debugger/how-to-set-debug-and-release-configurations?view=vs-2019

Running the Sample Client
-------------------------

Make sure the sensor is connected to the network. See "Connecting to the Sensor" in the `Software
User Manual <https://www.ouster.com/downloads>`_ for instructions and different options for network
configuration.

Navigate to ``ouster_client`` under the build directory, which should contain an executable named
``ouster_client_example``. This program will attempt to connect to the sensor, capture lidar data,
and write point clouds out to CSV files::

    ./ouster_client_example <sensor hostname> <udp data destination>

where ``<sensor hostname>`` can be the hostname (os-99xxxxxxxxxx) or IP of the sensor and ``<udp
data destingation>`` is the hostname or IP to which the sensor should send lidar data. You can also
supply ``""``, an empty string, to utilize automatic detection.

On Windows, you may need to allow the client/visualizer through the Windows firewall to receive
sensor data.

Running the Sample Visualizer
-----------------------------

Navigate to ``ouster_viz`` under the build directory, which should contain an executable named
``simple_viz`` . Run::

    ./simple_viz [flags] <sensor hostname> [udp data destination]

where ``<sensor hostname>`` can be the hostname (os-99xxxxxxxxxx) or IP of the sensor and ``[udp
data destingation]`` is an optional hostname or IP to which the sensor should send lidar data.

The sample visualizer does not currently include a GUI, but can be controlled with the mouse and
keyboard:

* Click and drag rotates the view
* Middle click and drag moves the view
* Scroll adjusts how far away the camera is from the vehicle

Keyboard controls:

    ============= ============================================
        key       what it does
    ============= ============================================
    ``p``         Increase point size
    ``o``         Decrease point size
    ``m``         Cycle point cloud coloring mode
    ``v``         Toggle range cycling
    ``n``         Toggle display near-IR image from the sensor
    ``shift + r`` Reset camera
    ``e``         Change size of displayed 2D images
    ``;``         Increase spacing in range markers
    ``'``         Decrease spacing in range markers
    ``r``         Toggle auto rotate
    ``w``         Camera pitch up
    ``s``         Camera pitch down
    ``a``         Camera yaw left
    ``d``         Camera yaw right
    ``1``         Toggle point cloud visibility
    ``0``         Toggle orthographic camera
    ``=``         Zoom in
    ``-``         Zoom out
    ``shift``     Camera Translation with mouse drag
    ============= ============================================

For usage and other options, run ``./simple_viz -h``


Example ROS Code
================

The sample code include tools for publishing sensor data as standard ROS topics. Since ROS uses its
own build system, it must be compiled separately from the rest of the sample code.

The provided ROS code has been tested on ROS Kinetic, Melodic, and Noetic on Ubuntu 16.04, 18.04,
and 20.04, respectively. Use the `installation instructions <https://www.ros.org/install/>`_ to get
started with ROS on your platform.

Building
--------

The build dependencies include those of the sample code::

    sudo apt install build-essential cmake libglfw3-dev libglew-dev libeigen3-dev \
         libjsoncpp-dev libtclap-dev

Additionally, you should install the ros dependencies::

    sudo apt install ros-<ROS-VERSION>-ros-core ros-<ROS-VERSION>-pcl-ros \
         ros-<ROS-VERSION>-tf2-geometry-msgs ros-<ROS-VERSION>-rviz

where ``<ROS-VERSION>`` is ``kinetic``, ``melodic``, or ``noetic``. 


Alternatively, if you would like to install dependencies with `rosdep`::

    rosdep install --from-paths <path to ouster example>

To build::

    source /opt/ros/<ROS-VERSION>/setup.bash
    mkdir -p ./myworkspace/src
    cd myworkspace
    ln -s <path to ouster_example> ./src/
    catkin_make -DCMAKE_BUILD_TYPE=Release

**Warning:** Do not create your workspace directory inside the cloned ouster_example repository, as
this will confuse the ROS build system.

For each command in the following sections, make sure to first set up the ROS environment in each
new terminal by running::

        source myworkspace/devel/setup.bash

Running ROS Nodes with a Sensor
-------------------------------

Make sure the sensor is connected to the network. See "Connecting to the Sensor" in the `Software
User Manual`_ for instructions and different options for network configuration.

To publish ROS topics from a running sensor, run::

    roslaunch ouster_ros ouster.launch sensor_hostname:=<sensor hostname> \
                                       metadata:=<path to metadata json>

where:

* ``<sensor hostname>`` can be the hostname (os-99xxxxxxxxxx) or IP of the sensor
* ``<path to metadata json>`` is the path to the json file to which to save calibration metadata

you can also optionally specify:

* ``udp_dest:=<hostname>`` to specify the hostname or IP to which the sensor should send data
* ``lidar_mode:=<mode>`` where mode is one of ``512x10``, ``512x20``, ``1024x10``, ``1024x20``, or
  ``2048x10``, and
* ``viz:=true`` to visualize the sensor output, if you have the rviz ROS package installed

Note that by default the working directory of all ROS nodes is set to ``${ROS_HOME}``, generally
``$HOME/.ros``, so if ``metadata`` is a relative path, it will write to that path inside
``${ROS_HOME}``. To avoid this, you can provide an absolute path to ``metadata``.

Recording Data
--------------

To record raw sensor output use `rosbag record`_. After starting the ``roslaunch`` command above, in
another terminal, run::

    rosbag record /os_node/imu_packets /os_node/lidar_packets

This will save a bag file of recorded data in the current working directory.

You should copy and save the metadata file alongside your data. The metadata file will be saved at
the provided path to `roslaunch`. If you run the node and cannot find the metadata file, try looking
inside your ``${ROS_HOME}``, generally ``$HOME/.ros``. Regardless, you must retain the metadata
file, as you will not be able to replay your data later without it.

.. _rosbag record: https://wiki.ros.org/rosbag/Commandline#rosbag_record

Playing Back Recorded Data
--------------------------

To publish ROS topics from recorded data, specify the ``replay`` and ``metadata`` parameters when
running ``roslaunch``::

    roslaunch ouster_ros ouster.launch replay:=true metadata:=<path to metadata json>

And in a second terminal run `rosbag play`_::

    rosbag play --clock <bag files ...>

A metadata file is mandatory for replay of data. See `Recording Data`_ for how
to obtain the metadata file when recording your data.

.. _rosbag play: https://wiki.ros.org/rosbag/Commandline#rosbag_play


Ouster Python SDK
=================

Python SDK users should proceed straight to the `Ouster python SDK homepage <python/>`_.


Additional Information
======================

* Sample sensor output usable with the provided ROS code `is available here
  <https://ouster.com/resources/lidar-sample-data>`_.
* For network configuration, refer to "Connecting to the Sensor" in the `Software User Manual`_.
