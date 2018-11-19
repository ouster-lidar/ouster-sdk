#!/bin/bash

xhost +local:root
IMG=wilselby/ros_melodic:cartographer_ros

# If NVIDIA is present, use Nvidia-docker
if test -c /dev/nvidia0
then
    docker run --rm -it \
      --runtime=nvidia \
      --privileged \
      --device /dev/dri:/dev/dri \
      --env="DISPLAY" \
      --env="QT_X11_NO_MITSHM=1" \
      -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      $IMG \
      bash
else
    docker run --rm -it \
      -e DISPLAY \
      --device=/dev/dri:/dev/dri \
      -v "/tmp/.X11-unix:/tmp/.X11-unix" \
      $IMG \
      bash
fi

# Google Cartographer commands

# Run all commands from the /root/bags directory
# cd /root/bags

## Validate rosbag
#rosrun cartographer_ros cartographer_rosbag_validate -bag_filename /root/bags/2018-09-26-09-01-00.bag

## Run 2D online
#roslaunch cartographer_ros demo_cart_2d.launch bag_filename:=/root/bags/2018-09-26-09-01-00.bag 

## Run 2D offline (to generate .pbstream file)
#roslaunch cartographer_ros offline_cart_2d.launch bag_filenames:=/root/bags/2018-09-26-09-01-00.bag 

## Run 2D assets_writer
#roslaunch cartographer_ros assets_writer_cart_2d.launch bag_filenames:=/root/bags/2018-09-26-09-01-00.bag  pose_graph_filename:=/root/bags/2018-09-26-09-01-00.bag.pbstream 

## View output pngs
#xdg-open 2018-09-26-09-01-00.bag_xray_xy_all.png
#xdg-open 2018-09-26-09-01-00.bag_probability_grid.png

## Run 3d online
#roslaunch cartographer_ros demo_cart_3d.launch bag_filename:=/root/bags/2018-09-26-09-01-00.bag 

## Run 3d offline
#roslaunch cartographer_ros offline_cart_3d.launch bag_filenames:=/root/bags/2018-09-26-09-01-00.bag 

## Run 3d assets_writer
#roslaunch cartographer_ros assets_writer_cart_3d.launch bag_filenames:=/root/bags/2018-09-26-09-01-00.bag  pose_graph_filename:=/root/bags/2018-09-26-09-01-00.bag.pbstream 

#xdg-open 2018-09-26-09-01-00.bag_xray_xy_all.png
#xdg-open 2018-09-26-09-01-00.bag_probability_grid.png

