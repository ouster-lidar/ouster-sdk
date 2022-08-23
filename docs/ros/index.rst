.. title:: ROS Guide

================
Example ROS Code
================

The sample code include tools for publishing sensor data as standard ROS topics. Since ROS uses
its own build system, it must be compiled separately from the rest of the sample code.

The provided ROS code has been tested on ROS Melodic on Ubuntu 18, and ROS Noetic Ubuntu 20. Use
the `installation instructions <http://wiki.ros.org/ROS/Installation>`_ to get started with ROS
on your platform.

Building ROS Driver
====================

The build dependencies include those of the sample code::

    sudo apt install build-essential cmake libeigen3-dev libjsoncpp-dev

Additionally, you should install the ros dependencies::

    sudo apt install \
      ros-<ROS-DISTRO>-pcl-ros \
      ros-<ROS-DISTRO>-tf2-geometry-msgs \
      ros-<ROS-DISTRO>-rviz

where ``<ROS-DISTRO>`` is ``melodic`` or ``noetic``.

Alternatively, if you would like to install dependencies with `rosdep`::

    rosdep install --from-paths <path to ouster example>

To build::

    mkdir -p ./catkin_ws/src
    cd catkin_ws
    ln -s <path to ouster_example> ./src/
    source /opt/ros/<ROS-VERSION>/setup.bash
    catkin_make -DCMAKE_BUILD_TYPE=Release

.. warning::
    Do not create your workspace directory inside the cloned ouster_example repository,
    as this will confuse the ROS build system.

For each command in the following sections, make sure to first set up the ROS environment in each
new terminal by running::

        source catkin_ws/devel/setup.bash

Running ROS Nodes with a Sensor
================================

Make sure the sensor is connected to the network. See "Connecting to the Sensor" in the `Software
User Manual <https://www.ouster.com/downloads>`_ for instructions and different options for network
configuration.

To connect to a sensor and publish its data as ROS topics, execute the command::

    roslaunch ouster_ros sensor.launch sensor_hostname:=<sensor hostname>

where:
- ``sensor_hostname:=<sensor hostname>`` can be the hostname (os-99xxxxxxxxxx.local) or IP of the
  sensor

Additionally, the launch file has following list of arguments that you can use:
- ``metadata:=<path-to-metadata>`` to set the name where sensor metadata configuration will be
  saved to. Note that by default the working directory of all ROS nodes is set to ``${ROS_HOME}``, 
  which is generally ``$HOME/.ros``. If you provide a relative path to ``metadata``, i.e.,
  ``metadata:=meta.json`` it will write to ``${ROS_HOME}/meta.json``. If you wish the file be saved 
  in the current directory you may use the absolute path instead, such as ``metadata:=$PWD/meta.json``
- ``udp_dest:=<hostname>`` to specify the hostname or IP to which the sensor should send data
- ``lidar_mode:=<mode>`` where mode is one of ``512x10``, ``512x20``, ``1024x10``, ``1024x20``, or
  ``2048x10``, and
- ``viz:=true/false`` to visualize the sensor output, if you have the rviz ROS package installed


Recording Data
===============

To record raw sensor output you may use the provided ``record.launch`` file as follows::

    roslaunch ouster_ros record.launch      \
        sensor_hostname:=<sensor hostname>  \
        metadata:=<json file name>          \
        bag_file:=<optional bag file name>

This will connect to the specified sensor, write the sensor metadata to a file and start
recording imu and lidar packets to the specified bag_file once the sensor is connected.

It is necessary that you provide a name for the metadata file and maintain this file along
with the recorded bag_file otherwise you won't be able to play the file correctly.

If no bag_file is specified then a name will be generated based on the current date/time.

By default ROS saves all files to $ROS_HOME, if you want to have these files saved in the
current directory, simply give the absolute path to each file. For example::

    roslaunch ouster_ros record.launch      \
        sensor_hostname:=<sensor hostname>  \
        metadata:=$PWD/<json file name>     \
        bag_file:=$PWD/<bag file name>

Alternatively, you may connect to the sensor using the ``roslaunch ouster_ros sensor.launch ..``
command and then use the rosbag command in a separate terminal to start recording lidar packets
at any time using the following command::

    rosbag record /ouster/imu_packets /ouster/lidar_packets

For more information on rosbag functionality refer to `rosbag record`_.

.. _rosbag record: https://wiki.ros.org/rosbag/Commandline#rosbag_record

.. warning::
    When recording a bag file directly via the ``rosbag record``, you need to
    save the metadata information of the sensor you are connected to. This can be
    achieved by supplying a path to the ``metadata`` argument of the ``sensor.launch``.
    You will need the metadata file information to properly replay the recorded bag
    file.

Playing Back Recorded Data
==========================

You may use the ``replay.launch`` file to repalay previously captured sensor data.
Simply invoke the launch file with the following parameters::

    roslaunch ouster_ros replay.launch      \
        metadata:=<json file name>          \
        bag_file:=<path to rosbag file>

A metadata file is mandatory for replay of data. See `Recording Data`_ for how
to obtain the metadata file when recording your data.

Ouster ROS Services
===================

The ROS driver currently advertises three services ``/ouster/get_metadata``,
``/ouster/get_config``, and ``/ouster/set_config``. The first one is available
in all three modes of operation: ``Sensor``, ``Replay``, and ``Recording``.
The latter two, however, are only available in ``Sensor`` and ``Recording``
modes. i.e. when connected to a sensor.

The usage of the three services is described below:

- ``/ouster/get_metadata``: This service takes no parameters and returns the
  current sensor metadata, you may use as follows::

    rosservice call /ouster/get_metadata

  This will return a json string that contains the sensor metadata

- ``/ouster/get_config``: This service takes no parameters and returns the
  current sensor configuration, you may use as follows::

    rosservice call /ouster/get_config

  This will return a json string represting the current configuration

- ``/ouster/set_config``: Takes a single parameter and also returns the updated
  sensor configuration. You may use as follows::

    rosservice call /ouster/set_config "config_file: '<path to sensor config>'"

  It is not guranteed that all requested configuration are applied to the sensor,
  thus it is the caller responsibilty to examine the returned json object and
  check which of the sensor configuration parameters were successfully applied.
