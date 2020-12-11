=========
Changelog
=========

[20201209]
==========

Changed
-------

* switched to date-based version scheme. No longer tracking firmware versions
* added a top-level ``CMakeLists.txt``. Client and visualizer should no longer be built
  separately. See the README for updated build instructions
* cmake cleanup, including using custom "find modules" to provide better compatibility between
  different versions of cmake
* respect standard cmake ``BUILD_SHARED_LIBS`` and ``CMAKE_POSITION_INDEPENDENT_CODE`` flags
* make ``ouster_ros`` easier to use as a dependency by bundling the client and viz libraries
  together into a single library that can be used through catkin
* updated client example code. Now uses more of the client APIs to capture data and write to a
  CSV. See ``ouster_client/src/example.cpp``
* replace callback-based ``batch_to_scan`` function with ``ScanBatcher``. See ``lidar_scan.h`` for
  API docs and the new client example code
* update ``LidarScan`` API. Now includes accessors for measurement blocks as well as channel data
  fields. See ``lidar_scan.h`` for API docs
* add client version field to metadata json, logs, and help text
* client API renaming to better reflect the Sensor Software Manual

[1.14.0-beta.14] - 2020-08-27
=============================

Added
-----

* support for ROS noetic in ``ouster_ros``. Note: this may break building on very old platforms
  without a C++14-capable compiler
* an extra extrinsics field in ``sensor_info`` for conveniently passing around an extra user-supplied
  transform
* a utility function to convert ``lidar_scan`` data between the "staggered" representation where each
  column has the same timestamp and "de-staggered" representation where each column has the same
  azimuth angle
* mask support in the visualizer library in ``ouster_viz``

Changed
-------

* ``ouster_ros`` now requires C++14 to support building against noetic libraries
* replaced ``batch_to_iter`` with ``batch_to_scan``, a simplified function that writes directly to a
  ``lidar_scan`` instead of arbitrary iterator

Fixed
-----

* ipv6 support using dual-stack sockets on all supported platforms. This was broken since the
  beta.10 release
* projection to Cartesian coordinates now takes into account the vertical offset the sensor and
  lidar frames
* the reference frame of point cloud topics in ``ouster_ros`` is now correctly reported as the "sensor
  frame" defined in the user guide

[1.14.0-beta.12] - 2020-07-10
=============================

*no changes*

[1.14.0-beta.11] - 2020-06-17
=============================

*no changes*

[1.14.0-beta.10] - 2020-05-21
=============================

Added
-----

* preliminary support for Windows and macOS for ``ouster_viz`` and ``ouster_client``

Changed
-------

* replaced VTK visualizer library with one based on GLFW
* renamed all instances of "OS1" including namespaces, headers, node and topic names, to reflect
  support for other product lines
* updated all xyz point cloud calculations to take into account new ``lidar_origin_to_beam_origin``
  parameter reported by sensors
* client and ``os_node`` and ``simple_viz`` now avoid setting the lidar and timestamp modes when
  connecting to a client unless values are explicitly specicified

Fixed
-----

* increase the UDP receive buffer size in the client to reduce chances of dropping packets on
  platforms with low defaults
* ``os_cloud_node`` output now uses the updated point cloud calculation, taking into account the lidar
  origin offset
* minor regression with destaggering in img_node output in previous beta

[1.14.0-beta.4] - 2020-03-17
============================

Added
-----

* support for gen2 hardware in client, visualizer, and ROS sample code
* support for updated "packed" lidar UDP data format for 16 and 32-beam devices with firmware 1.14
* range markers in ``simple_viz`` and ``viz_node``. Toggle display using ``g`` key. Distances can be
  configured from ``os1.launch``.
* post-processing to improve ambient image uniformity in visualizer

Changed
-------

* use random ports for lidar and imu data by default when unspecified

[1.13.0] - 2020-03-16
=====================

Added
-----

* post-processing to improve ambient image uniformity in visualizer
* make timestamp mode configurable via the client (PR #97)

Changed
-------

* turn on position-independent code by default to make using code in libraries easier (PR #65)
* use random ports for lidar and imu data by default when unspecified

Fixed
-----

* prevent legacy tf prefix from making invalid frame names (PR #56)
* use ``iterator_traits`` to make ``batch_to_iter`` work with more types (PR #70)
* use correct name for json dependency in ``package.xml`` (PR #116)
* handle udp socket creation error gracefully in client

[1.12.0] - 2019-05-02
=====================

Added
-----

* install directives for ``ouster_ros`` build (addresses #50)

Changed
-------

* flip the sign on IMU acceleration output to follow usual conventions
* increase the update rate in the visualizer to ~60hz

Fixed
-----

* visualizer issue where the point cloud would occasionally occasionally not be displayed using
  newer versions of Eigen

[1.11.0] - 2019-03-26
=====================

Added
-----

* allow renaming tf ids using the ``tf_prefix`` parameter

Changed
-------

* use frame id to batch packets so client code deals with reordered lidar packets without splitting
  frames
* use a uint32_t for PointOS1 timestamps to avoid unnecessary loss of precision

Fixed
-----

* bug causing ring and reflectivity to be corrupted in os1_cloud_node output
* misplaced sine in azimuth angle calculation (addresses #42)
* populate timestamps on image node output (addresses #39)

[1.10.0] - 2019-01-27
=====================

Added
-----

* ``os1_node`` now queries and uses calibrated beam angles from the sensor
* ``os1_node`` now queries and uses imu / lidar frames from the sensor
* ``os1_node`` reads and writes metadata to ``${ROS_HOME}`` to support replaying data with calibration
* ROS example code now publishes tf2 transforms for imu / lidar frames (addresses #12)
* added ``metadata`` parameter to ``os1.launch`` to override location of metadata
* added ``viz`` parameter to ``os1.launch`` to run the example visualizer with ROS
* added ``image`` parameter to ``os1.launch`` to publish image topics to rviz (addresses #21)
* added range field to ``PointOS1``

Changed
-------

* split point-cloud publishing out of ``os1_node`` into ``os1_cloud_node``
* example visualizer controls:

  - press ``m`` to cycle through color modes instead of ``i``, ``z``, ``Z``, ``r``
  - ``r`` now resets the camera position
  - range/signal images automatically resized to fit window height

* updated OS-1 client to use newer TCP configuration commands
* updated OS-1 client to set the requested lidar mode, reinitialize on connection
* changed point cloud batching to be based on angle rather than scan duration
* ``ouster_client`` now depends on the ``jsoncpp`` library
* switched order of fields in ``PointOS1`` to be compatible with ``PointXYZI`` (addresses #16)
* moved example visualizer VTK rendering into the main thread (merged #23)
* the timestamp field of PointOS1 now represents time since the start of the scan (the timestamp of
  the PointCloud2 message) in nanoseconds

Removed
-------

* removed keyboard camera controls in example visualizer
* removed panning and rotating of the image panel in example visualizer

Fixed
-----

* no longer dropping UDP packets in 2048 mode on tested hardware
* example visualizer:

  - point cloud display focus no longer snaps back on rotation
  - fixed clipping issues with parallel projection
  - fixed point coloring issues in z-color mode
  - improved visualizer performance
