# ouster_ros

## Requirements
- Ubuntu 18|20
- ROS Melodic|Noetic

## Usage

### Sensor Mode
```bash
roslaunch ouster_ros sensor.launch      \
    sensor_hostname:=<sensor host name> \
    metadata:=<json file name>
```

### Replay Mode
```bash
roslaunch ouster_ros replay.launch      \
    metadata:=<json file name>          \
    bag_file:=<path to rosbag file>
```

### Recording Mode
```bash
roslaunch ouster_ros record.launch      \
    sensor_hostname:=<sensor host name> \
    metadata:=<json file name>          \
    bag_file:=<optional bag file name>
```

## Services
The ROS driver currently advertises three services `/ouster/get_metadata`,
`/ouster/get_config`, and `/ouster/set_config`. The first one is available
in all three modes of operation: Sensor, Replay, and Recording. The latter two,
however, are only available in Sensor and Recording modes. i.e. when connected
to a sensor.

The usage of the three services is described below:
* `/ouster/get_metadata`: This service takes no parameters and returns the
current sensor metadata, you may use as follow:
```bash
rosservice call /ouster/get_metadata "{}"
```
This will return a json string that contains the sensor metadata

* `/ouster/get_config`: This service takes no parameters and returns the
current sensor configuration, you may use as follow:
```bash
rosservice call /ouster/get_config "{}"
```
This will return a json string represting the current configuration

* `/ouster/set_config`: Takes a single parameter and also returns the updated
sensor configuration. You may use as follows:
```bash
rosservice call /ouster/set_config "config_file: '<path to sensor config>'"
```
It is not guranteed that all requested configuration are applied to the sensor,
thus it is the caller responsibilty to examine the returned json object and
check which of the sensor configuration parameters were successfully applied.