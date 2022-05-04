.. title:: ROS Guide

================
Example ROS Code
================

The sample code include tools for publishing sensor data as standard ROS topics. Since ROS uses its
own build system, it must be compiled separately from the rest of the sample code.

The provided ROS code has been tested on ROS Kinetic, Melodic, and Noetic on Ubuntu 16.04, 18.04,
and 20.04, respectively. Use the `installation instructions <http://wiki.ros.org/ROS/Installation>`_ to get
started with ROS on your platform.

Building ROS Driver
====================

The build dependencies include those of the sample code::

    sudo apt install build-essential cmake libeigen3-dev libjsoncpp-dev

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
================================

Make sure the sensor is connected to the network. See "Connecting to the Sensor" in the `Software
User Manual <https://www.ouster.com/downloads>`_ for instructions and different options for network
configuration.

To publish ROS topics from a running sensor, run::

    roslaunch ouster_ros ouster.launch sensor_hostname:=<sensor hostname> \
                                       metadata:=<path to metadata json>

where:

* ``<sensor hostname>`` can be the hostname (os-99xxxxxxxxxx) or IP of the sensor
* ``<path to metadata json>`` is the path you want to save sensor metadata to.
  You must provide a JSON filename at the end, not just a path to a directory.

Note that by default the working directory of all ROS nodes is set to ``${ROS_HOME}``, generally
``$HOME/.ros``. If you provide a relative path to ``metadata``, i.e., ``metadata:=meta.json``, it 
will write to ``${ROS_HOME}/meta.json``. To avoid this, you can provide an absolute path to 
``metadata``, i.e. ``metadata:=/home/user/meta.json``.

You can also optionally specify:

* ``udp_dest:=<hostname>`` to specify the hostname or IP to which the sensor should send data
* ``lidar_mode:=<mode>`` where mode is one of ``512x10``, ``512x20``, ``1024x10``, ``1024x20``, or
  ``2048x10``, and
* ``viz:=true`` to visualize the sensor output, if you have the rviz ROS package installed


Recording Data
===============

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
==========================

To publish ROS topics from recorded data, specify the ``replay`` and ``metadata`` parameters when
running ``roslaunch``::

    roslaunch ouster_ros ouster.launch replay:=true metadata:=<path to metadata json>

And in a second terminal run `rosbag play`_::

    rosbag play --clock <bag files ...>

A metadata file is mandatory for replay of data. See `Recording Data`_ for how
to obtain the metadata file when recording your data.

.. _rosbag play: https://wiki.ros.org/rosbag/Commandline#rosbag_play
