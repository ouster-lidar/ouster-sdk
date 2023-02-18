=========
Changelog
=========

[unreleased]
============

ouster_client
-------------
- Added a new method ``mtp_init_client`` to init the client with multicast support (experimental).


[20230114]
==========

ouster_client
--------------
* breaking change: signal multiplier type changed to double to support new FW values of signal
  multiplier.
* breaking change: make_xyz_lut takes mat4d beam_to_lidar_transform instead of
  lidar_origin_to_beam_origin_mm double to accomodate new FWs. Old reference Python implementation
  was kept, but new reference was also added.
* address an issue that could cause the processed frame being dropped in favor or the previous
  frame when the frame_id wraps-around.
* added a new flag ``CONFIG_FORCE_REINIT`` for ``set_config()`` method, to force the sensor to reinit
  even when config params have not changed.
* breaking change: drop defaults parameters from the shortform ``init_client()`` method.
* added a new method ``init_logger()`` to provide control over the logs emitted by ``ouster_client``.
* add parsing for new FW 3.0 thermal features shot_limiting and thermal_shutdown statuses and countdowns
* add frame_status to LidarScan
* introduce a new method ``cartesianT()`` which speeds up the computation of point projecion from range
  image, the method also can process the cartesian product with single float precision. A new unit test
  ``cartesian_test`` which shows achieved speed up gains by the number of valid returns in lidar scan.
* add ``RAW_HEADERS`` ChanField to LidarScan for packing headers and footer (alpha version, may be
  changed/removed without notice in the future)

python
------
* breaking change: drop defaults parameters of ``client.Sensor`` constructor.
* breaking change: change Scans interface Timeout to default to 1 second instead of None
* added a new method ``init_logger()`` to provide control over the logs emitted by ``client.Sensor``.
* add ``client.LidarScan`` methods ``__repr__()`` and ``__str__()``.
* changed default timeout from 1 seconds to 2 seconds

ouster_viz
----------
* add ``SimpleViz.screenshot()`` function and a key handler ``SHIFT-Z`` to
  save a screenshot. Can be called from a client thread, and executes
  asyncronously (i.e. returns immediately and pushes a one off callback
  to frame buffer handlers)
* add ``PointViz.viewport_width()`` and ``PointViz.viewport_height()`` functions
* add ``PointViz.push/pop_frame_buffer_handler()`` to attach a callbacks on
  every frame draw update that calls from the main rendering loop.
* add ``SHIFT-X`` key to SimpleViz to start continuous saving of screenshots
  on every draw operation. (good for making videos)
* expose ``Camera.set_target`` function through pybind

ouster-sdk
----------
* Moved ouster_ros to its own repo
* pin ``openssl`` Conan package dependency to ``openssl/1.1.1s`` due to
  ``libtins`` and ``libcurl`` conflicting ``openssl`` versions


[20220927]
==========

ouster_client
--------------
* fix a bug in longform ``init_client()`` which was not setting timestamp_mode and lidar_mode correctly
  

[20220826]
==========

* drop support for buliding C++ libraries and Python bindings on Ubuntu 16.04
* drop support for buliding C++ libraries and Python bindings on Mac 10.13, Mac 10.14
* Python 3.6 wheels are no longer built and published
* drop support for sensors running FW < 2.0
* require C++ 14 to build

ouster_client
--------------
* add ```CUSTOM0-9`` ChanFields to LidarScan object
* fix parsing measurement status from packets: previously, with some UDP profiles, higher order bits
  could be randomly set
* add option for EIGEN_MAX_ALIGN_BYTES, ON by default
* use of sensor http interface for comms with sensors for FW 2.1+
* propogate C++ 17 usage requirement in cmake for C++ libraries built as C++17
* allow vcpkg configuration via environment variables
* fix a bug in sensor_config struct equality comparison operator

ouster_pcap
-----------
* fix incorrect encapsulation protocol being reported in ``packet_info``

ouster_viz
----------
* clean up GL context logic to avoid errors on window/intel UHD graphics

python
------
* windows extension modules are now statically linked to avoid potential issues with vendored dlls

ouster_ros
----------
* drop ROS kinetic support
* switch from nodes to nodelets
* update topic names, group under single ros namespace
* separate launch files for play, replay, and recording
* drop FW 1.13 compatibility for sensors and recorded bags
* remove setting of EIGEN_MAX_ALIGN_BYTES
* add two new ros services /ouster/get_config and /ouster/set_config (experimental)
* add new timestamp_mode TIME_FROM_ROS_TIME
* declare PCL_NO_PRECOMPILE ahead of all PCL library includes


[20220608]
==========

ouster_client
-------------
* change single return parsing for FW 2.3.1

python
------
* single return parsing for FW 2.3.1 reflects change from ouster_client


[20220504]
==========

* update supported vcpkg tag to 2022.02.23
* update to manylinux2014 for x64 linux ``ouster-sdk`` wheels
* Ouster SDK documentation overhaul with C++/Python APIs in one place
* sample data updated to firmware 2.3

ouster_client
-------------
* fix the behavior of ``BeamUniformityCorrector`` on azimuth-windowed data by ignoring zeroed out
  columns
* add overloads in ``image_processing.h`` to work with single-precision floats
* add support for new ``RNG19_RFL8_SIG16_NIR16`` single-return and ``RNG15_RFL8_NIR8`` low-bandwidth
  lidar UDP profiles introduced in firmware 2.3

ouster_viz
----------
* switch to glad for OpenGL loading. GLEW is still supported for developer builds
* breaking change: significant API update of the ``PointViz`` library. See documentation for details
* the ``simple_viz`` example app and ``LidarScanViz`` utility have been removed. Equivalent
  functionality is now provided via Python
* add basic support for drawing 2d and 3d text labels
* update to OpenGL 3.3

python
------
* fix a bug where incorrectly sized packets read from the network could cause the client thread to
  silently exit, resulting in a timeout
* fix ``client.Scans`` not raising a timeout when using the ``complete`` flag and receiving only
  incomplete scans. This could cause readings scans to hang in rare situations
* added bindings for the new ``PointViz`` API and a new module for higher-level visualizer utilities
  in ``ouster.sdk.viz``. See API documentation for details
* the ``ouster-sdk`` package now includes an example visualizer, ``simple-viz``, which will be
  installed on that path for the Python environment

ouster_ros
-----------
* support new fw 2.3 profiles by checking for inclusion of fields when creating point cloud. Missing
  fields are filled with zeroes

[20220107]
==========

* add support for arm64 macos and linux. Releases are now built and tested on these platforms
* add support for Python 3.10
* update supported vcpkg tag to 2021.05.12
* add preliminary cpack and install support. It should be possible to use a pre-built SDK package
  instead of including the SDK in the build tree of your project

ouster_client
-------------
* update cmake package version to 0.3.0
* avoid unnecessary DNS lookup when using numeric addresses with ``init_client()``
* disable collecting metadata when sensor is in STANDBY mode
* breaking change: ``set_config()`` will now produce more informative errors by throwing
  ``std::invalid_argument`` with an error message when config parameters fail validation
* use ``SO_REUSEPORT`` for UDP sockets on non-windows platforms
* the set of fields available on ``LidarScan`` is now configurable. See the new ``LidarScan``
  constructors for details
* added ``RANGE2``, ``SIGNAL2`` and ``REFLECTIVITY2`` channel fields to support handling data from
  the second return
* ``ScanBatcher`` will now parse and populate only the channel fields configured on the
  ``LidarScan`` passed to ``operator()()``
* add support for new configuration parameters: ``udp_profile_lidar``, ``udp_profile_imu`` and
  ``columns_per_packet``
* add udp ports, the new initialization id field, and udp profiles to the metadata stored in
  the ``sensor_info`` struct
* ``sensor_info::name`` is now deprecated and will stop being populated in the future
* add methods to query and iterate over available ``LidarScan`` fields and field types
* breaking change: removed ``LidarScan::block`` and ``LidarScan::data`` members. These can't be
  supported for different packet profiles
* the ``LidarScan::Field`` defniition has been moved to ``sensor::ChanField`` and enumerators have
  been renamed to match the sensor user manual. The old names are still available, but deprecated
* deprecate accessing encoder values and frame ids from measurement blocks using ``packet_format``
  as these will not be reported by the sensor in some future configurations
* add ``packet_frame_id`` member function to ``packet_format``
* add ``col_field`` member function to ``packet_format`` for parsing channel field values for an
  entire measurement block
* add new accessors for measurement headers to ``LidarScan``, deprecating the existing ``header``
  member function
* represent empty sensor config with an empty object instead of null in json representation of the
  ``sensor_config`` datatype
* update cmake package version to 0.2.1
* add a conservative socket read timeout so ``init_client()`` will fail with an error message when
  another client fails to close a TCP connection (addresses #258)
* when passed an empty string for the ``udp_dest_host`` parameter, ``init_client()`` will now
  configure the sensor using ``set_udp_dest_auto``. Previously, this would turn off UDP output on
  the sensor, so any attempt to read data would time out (PR #255)
* fall back to binding ipv4 UDP sockets when ipv6 is not available (addresses #261)

ouster_pcap
-----------
* report additional information in the ``packet_info`` struct and remove separate ``stream_info``
  API
* switch the default pcap encapsulation to ethernet for Ouster Studio compatibility (addresses #265)

ouster_ros
----------
* update ROS package version to 0.3.0
* allow setting the packet profile in ouster.launch with the ``udp_profile_lidar`` parameter
* publish additional cloud and image topics for the second return when running in dual returns mode
* fix ``os_node`` crash on shutdown due to Eigen alignment flag not being propogated by catkin
* update ROS package version to 0.2.1
* the ``udp_dest`` parameter to ouster.launch is now optional when connecting to a sensor

ouster_viz
----------
* the second CLI argument of simple_viz specifying the UDP data destination is now optional
* fixed bug in AutoExposure causing more points to be mapped to near-zero values
* add functionality to display text over cuboids

python
------
* update ouster-sdk version to 0.3.0
* improve heuristics for identifying sensor data in pcaps, including new packet formats
* release builds for wheels on Windows now use the VS 2017 toolchain and runtime (previously 2019)
* fix potential use-after-free in ``LidarScan.fields``
* update ouster-sdk version to 0.3.0b1
* return an error when attempting to initialize ``client.Sensor`` in STANDBY mode
* check for errors while reading from a ``Sensor`` packet source and waiting for a timeout. This
  should make stopping a process with ``SIGINT`` more reliable
* add PoC bindings for the ``ouster_viz`` library with a simple example driver. See the
  ``ouster.sdk.examples.viz`` module
* add bindings for new configuration and metadata supported by the client library
* breaking change: the ``ChanField`` enum is now implemented as a native binding for easier interop
  with C++. Unlike Python enums, the bound class itself is no longer sized or iterable. Use
  ``ChanField.values`` to iterate over all ``ChanField`` values or ``LidarScan.fields`` for fields
  available on a particular scan instance
* breaking change: arrays returned by ``LidarPacket.field`` and ``LidarPacket.header`` are now
  immutable. Modifying the underlying packet buffer through these views was never fully supported
* deprecate ``ColHeader``, ``LidarPacket.header``, and ``LidarScan.header`` in favor of new
  properties: ``timestamp``, ``measurement_id``, ``status``, and ``frame_id``
* replace ``LidarScan`` with native bindings implementing the same API
* ``xyzlut`` can now accept a range image as an ndarray, not just a ``LidarScan``
* update ouster-sdk version to 0.2.2
* fix open3d example crash on exit when replaying pcaps on macos (addresses #267)
* change open3d normalization to use bound AutoExposure


[20210608]
==========

ouster_client
-------------
* update cmake package version to 0.2.0
* add support for new signal multiplier config parameter
* add early version of a C++ API covering the full sensor configuration interface
* increase default initialization timeout to 60 seconds to account for the worst case: waking up
  from STANDBY mode

ouster_pcap
-----------
* ``record_packet()`` now requires passing in a capture timestamp instead of using current time
* work around libtins issue where capture timestamps for pcaps recorded on Windows are always zero
* add preliminary C++ API for working with pcap files containing a single sensor packet capture

ouster_ros
----------
* update ROS package version to 0.2.0
* add Dockerfile to easily set up a build environment or run nodes
* ``img_node`` now outputs 16-bit images, which should be more useful. Range image output is now in
  units of 4mm instead of arbitrary scaling (addresses #249)
* ``img_node`` now outputs reflectivity images as well on the ``reflec_image`` topic
* change ``img_node`` topics to match terminology in sensor documentation: ``ambient_image`` is now
  ``nearir_image`` and ``intensity_image`` is now ``signal_image``
* update rviz config to use flat squares by default to work around `a bug on intel systems
  <https://github.com/ros-visualization/rviz/issues/1508>`_
* remove viz_node and all graphics stack dependencies from the package. The ``viz`` flag on the
  launch file now runs rviz (addresses #236)
* clean up package.xml and ensure that dependencies are installable with rosdep (PR #219)
* the ``metadata`` argument to ouster_ros launch file is now required. No longer defaults to a name
  based on the hostname of the sensor

ouster_viz
----------
* update reflectivity visualization for changes in the upcoming 2.1 firmware. Add new colormap and
  handle 8-bit reflectivity values
* move most of the visualizer code out of public headers and hide some implementation details
* fix visualizer bug causing a small viewport when resizing the window on macos with a retina
  display

python
------
* update ouster-sdk version to 0.2.1
* fix bug in determining if a scan is complete with single-column azimuth windows
* closed PacketSource iterators will now raise an exception on read
* add examples for visualization using open3d (see: ``ouster.sdk.examples.open3d``)
* add support for the new signal multiplier config parameter
* preserve capture timestamps on packets read from pcaps
* first release: version 0.2.0 of ouster-sdk. See the README under the ``python`` directory for
  details and links to documentation


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
