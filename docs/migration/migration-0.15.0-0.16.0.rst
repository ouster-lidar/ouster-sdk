================================
Migration from 0.15.X to 0.16.0
================================

The OusterSDK 0.16.0 release introduces several breaking changes and deprecations. This guide summarizes how to migrate your code from 0.15.X to 0.16.0.

How to Identify Deprecated Code
+++++++++++++++++++++++++++++++

1. **Compiler Warnings**: The SDK uses deprecation attributes, so your compiler will warn you when you use deprecated APIs.

2. **Search for Deprecated Code**: You can search your codebase for the old to find code that needs to be updated.

3. **Check Deprecation Markers**: In the SDK headers, look for:

   .. code-block:: cpp

      OUSTER_DEPRECATED_TYPE(old_name, NewName, OUSTER_DEPRECATED_LAST_SUPPORTED_0_16)
      OUSTER_DEPRECATED_CONSTEXP(old_name, NEW_NAME, OUSTER_DEPRECATED_LAST_SUPPORTED_0_16)
      OUSTER_DEPRECATED_MSG(NEW_SYMBOL, OUSTER_DEPRECATED_LAST_SUPPORTED_0_16)
      OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(old_symbol, new_symbol, OUSTER_DEPRECATED_LAST_SUPPORTED_0_16)

Timeline for Removal
++++++++++++++++++++

All APIs marked with ``OUSTER_DEPRECATED_LAST_SUPPORTED_0_16`` will be removed in version 1.0.0. It is recommended to update your code as soon as possible to ensure a smooth transition to future SDK versions.

Naming Convention Changes
+++++++++++++++++++++++++

The Ouster SDK is standardizing naming and namespace conventions across the codebase.

All deprecated items that are marked with the ``OUSTER_DEPRECATED_LAST_SUPPORTED_0_16`` macros
will be removed in 1.0.0.

API Changes
+++++++++++

CPP Changes
-----------

Most items were moved under their namespace associated with their project in order to match the conventions already used in our Python codebase.

- Most items in ouster_client got moved into ``ouster::sdk::core``
- ``open_source`` and ``open_packet_source`` along with some associated classes got moved into ``ouster::sdk``
- Items in ouster_sensor got moved into ``ouster::sdk::sensor``
- Items in ouster_mapping got moved into ``ouster::sdk::mapping``
- Items in ouster_pcap got moved into ``ouster::sdk::pcap``
- Items in ouster_osf got moved into ``ouster::sdk::osf``
- Multiple enumerations were renamed.

In addition several types were renamed for naming consistency as follows:

ouster_client
^^^^^^^^^^^^^

+-----------------------+-----------------------+-------------------+
| Deprecated (0.16)     | Use Instead           | Namespaces        |
+=======================+=======================+===================+
| ``lidar_mode``        | ``LidarMode``         | ouster::sdk::core |
+-----------------------+-----------------------+-------------------+
| ``timestamp_mode``    | ``TimestampMode``     | ouster::sdk::core |
+-----------------------+-----------------------+-------------------+
| ``data_format``       | ``DataFormat``        | ouster::sdk::core |
+-----------------------+-----------------------+-------------------+
| ``calibration_status``| ``CalibrationStatus`` | ouster::sdk::core |
+-----------------------+-----------------------+-------------------+
| ``product_info``      | ``ProductInfo``       | ouster::sdk::core |
+-----------------------+-----------------------+-------------------+
| ``packet_format``     | ``PacketFormat``      | ouster::sdk::core |
+-----------------------+-----------------------+-------------------+
| ``range_unit``        | ``RANGE_UNIT``        | ouster::sdk::core |
+-----------------------+-----------------------+-------------------+
| ``version``           | ``Version``           | ouster::sdk::core |
+-----------------------+-----------------------+-------------------+
| ``invalid_version``   | ``INVALID_VERSION``   | ouster::sdk::core |
+-----------------------+-----------------------+-------------------+

ouster_sensor
^^^^^^^^^^^^^
+-----------------------+-----------------------+---------------------+
| Deprecated (0.16)     | Use Instead           | Namespaces          |
+=======================+=======================+=====================+
| ``client``            | ``Client``            | ouster::sdk::sensor |
+-----------------------+-----------------------+---------------------+
| ``client_state``      | ``ClientState``       | ouster::sdk::sensor |
+-----------------------+-----------------------+---------------------+
| ``min_version``       | ``MIN_VERSION``       | ouster::sdk::sensor |
+-----------------------+-----------------------+---------------------+

ouster_pcap
^^^^^^^^^^^

+--------------------+--------------------+-------------------+
| Deprecated (0.16)  | Use Instead        | Namespaces        |
+====================+====================+===================+
| ``packet_info``    | ``PacketInfo``     | ouster::sdk::pcap |
+--------------------+--------------------+-------------------+
| ``stream_key``     | ``StreamKey``      | ouster::sdk::pcap |
+--------------------+--------------------+-------------------+
| ``guessed_ports``  | ``GuessedPorts``   | ouster::sdk::pcap |
+--------------------+--------------------+-------------------+
| ``stream_data``    | ``StreamData``     | ouster::sdk::pcap |
+--------------------+--------------------+-------------------+
| ``playback_handle``| ``PlaybackHandle`` | ouster::sdk::pcap |
+--------------------+--------------------+-------------------+
| ``record_handle``  | ``RecordHandle``   | ouster::sdk::pcap |
+--------------------+--------------------+-------------------+
| ``packet_writer``  | ``PacketWriter``   | ouster::sdk::pcap |
+--------------------+--------------------+-------------------+

ouster_osf
^^^^^^^^^^

+-----------------+--------------------+--------------------+
| Deprecated (0.16) | Use Instead        | Namespaces       |
+=================+====================+====================+
| ``OSF_VERSION`` | ``OsfVersionString`` | ouster::sdk::osf |
+-----------------+--------------------+--------------------+

ouster_viz
^^^^^^^^^^

+---------------------------+---------------------------+------------------+
| Deprecated (0.16)         | Use Instead               | Namespaces       |
+===========================+===========================+==================+
| ``identity4d``            | ``IDENTITY4D``            | ouster::sdk::viz |
+---------------------------+---------------------------+------------------+
| ``default_window_width``  | ``DEFAULT_WINDOW_WIDTH``  | ouster::sdk::viz |
+---------------------------+---------------------------+------------------+
| ``default_window_height`` | ``DEFAULT_WINDOW_HEIGHT`` | ouster::sdk::viz |
+---------------------------+---------------------------+------------------+

Class Member Changes
^^^^^^^^^^^^^^^^^^^^

Several c++ classes have deprecated members:

* In LidarPacket, ImuPacket, and ZonePacket: ``my_type`` is deprecated. Use ``MY_TYPE`` instead.
* In field type classes: ``tag`` is deprecated. Use ``TAG`` instead.
* In OSF MessageHeader: ``setId()`` is deprecated. Use ``set_id()`` instead.

Enum deprecations
^^^^^^^^^^^^^^^^^

+---------------------------------------------+---------------------------------------+------------------------------------------+
| Deprecated (0.16)                           | Use Instead                           | Namespaces                               |
+=============================================+=======================================+==========================================+
| ``MODE_UNSPEC``                             | ``UNSPECIFIED``                       | ouster::sdk::core::LidarMode             |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``MODE_512x10``                             | ``_512x10``                           | ouster::sdk::core::LidarMode             |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``MODE_512x20``                             | ``_512x20``                           | ouster::sdk::core::LidarMode             |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``MODE_1024x10``                            | ``_1024x10``                          | ouster::sdk::core::LidarMode             |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``MODE_1024x20``                            | ``_1024x20``                          | ouster::sdk::core::LidarMode             |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``MODE_2048x10``                            | ``_2048x10``                          | ouster::sdk::core::LidarMode             |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``MODE_4096x5``                             | ``_4096x5``                           | ouster::sdk::core::LidarMode             |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``TIME_FROM_UNSPEC``                        | ``UNSPECIFIED``                       | ouster::sdk::core::TimestampMode         |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``OPERATING_UNSPEC``                        | ``UNSPECIFIED``                       | ouster::sdk::core::OperatingMode         |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``OPERATING_NORMAL``                        | ``NORMAL``                            | ouster::sdk::core::OperatingMode         |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``OPERATING_STANDBY``                       | ``STANDBY``                           | ouster::sdk::core::OperatingMode         |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``MULTIPURPOSE_OFF``                        | ``OFF``                               | ouster::sdk::core::MultipurposeIOMode    |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``MULTIPURPOSE_INPUT_NMEA_UART``            | ``INPUT_NMEA_UART``                   | ouster::sdk::core::MultipurposeIOMode    |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``MULTIPURPOSE_OUTPUT_FROM_INTERNAL_OSC``   | ``OUTPUT_FROM_INTERNAL_OSC``          | ouster::sdk::core::MultipurposeIOMode    |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``MULTIPURPOSE_OUTPUT_FROM_SYNC_PULSE_IN``  | ``OUTPUT_FROM_SYNC_PULSE_IN``         | ouster::sdk::core::MultipurposeIOMode    |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``MULTIPURPOSE_OUTPUT_FROM_PTP_1588``       | ``OUTPUT_FROM_PTP_1588``              | ouster::sdk::core::MultipurposeIOMode    |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``MULTIPURPOSE_OUTPUT_FROM_ENCODER_ANGLE``  | ``OUTPUT_FROM_ENCODER_ANGLE``         | ouster::sdk::core::MultipurposeIOMode    |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``POLARITY_ACTIVE_LOW``                     | ``ACTIVE_LOW``                        | ouster::sdk::core::Polarity              |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``POLARITY_ACTIVE_HIGH``                    | ``ACTIVE_HIGH``                       | ouster::sdk::core::Polarity              |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``PROFILE_LIDAR_UNKNOWN``                   | ``UNKNOWN``                           | ouster::sdk::core::UDPProfileLidar       |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``PROFILE_LIDAR_LEGACY``                    | ``LEGACY``                            | ouster::sdk::core::UDPProfileLidar       |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL``     | ``RNG19_RFL8_SIG16_NIR16_DUAL``       | ouster::sdk::core::UDPProfileLidar       |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``PROFILE_RNG19_RFL8_SIG16_NIR16``          | ``RNG19_RFL8_SIG16_NIR16``            | ouster::sdk::core::UDPProfileLidar       |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``PROFILE_RNG15_RFL8_NIR8``                 | ``RNG15_RFL8_NIR8``                   | ouster::sdk::core::UDPProfileLidar       |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``PROFILE_FIVE_WORD_PIXEL``                 | ``FIVE_WORD_PIXEL``                   | ouster::sdk::core::UDPProfileLidar       |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL``       | ``FUSA_RNG15_RFL8_NIR8_DUAL``         | ouster::sdk::core::UDPProfileLidar       |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``PROFILE_RNG15_RFL8_NIR8_DUAL``            | ``RNG15_RFL8_NIR8_DUAL``              | ouster::sdk::core::UDPProfileLidar       |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``PROFILE_RNG15_RFL8_NIR8_ZONE16``          | ``RNG15_RFL8_NIR8_ZONE16``            | ouster::sdk::core::UDPProfileLidar       |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``PROFILE_RNG19_RFL8_SIG16_NIR16_ZONE16``   | ``RNG19_RFL8_SIG16_NIR16_ZONE16``     | ouster::sdk::core::UDPProfileLidar       |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``PROFILE_LIDAR_OFF``                       | ``OFF``                               | ouster::sdk::core::UDPProfileLidar       |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``PROFILE_IMU_LEGACY``                      | ``LEGACY``                            | ouster::sdk::core::UDPProfileIMU         |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``PROFILE_ACCEL32_GYRO32_NMEA``             | ``ACCEL32_GYRO32_NMEA``               | ouster::sdk::core::UDPProfileIMU         |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``PROFILE_IMU_OFF``                         | ``OFF``                               | ouster::sdk::core::UDPProfileIMU         |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``FSR_NORMAL``                              | ``NORMAL``                            | ouster::sdk::core::FullScaleRange        |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``FSR_EXTENDED``                            | ``EXTENDED``                          | ouster::sdk::core::FullScaleRange        |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``ORDER_STRONGEST_TO_WEAKEST``              | ``STRONGEST_TO_WEAKEST``              | ouster::sdk::core::ReturnOrder           |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``ORDER_FARTHEST_TO_NEAREST``               | ``FARTHEST_TO_NEAREST``               | ouster::sdk::core::ReturnOrder           |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``ORDER_NEAREST_TO_FARTHEST``               | ``NEAREST_TO_FARTHEST``               | ouster::sdk::core::ReturnOrder           |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``ORDER_DEPRECATED_STRONGEST_RETURN_FIRST`` | ``DEPRECATED_STRONGEST_RETURN_FIRST`` | ouster::sdk::core::ReturnOrder           |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``ORDER_DEPRECATED_LAST_RETURN_FIRST``      | ``DEPRECATED_LAST_RETURN_FIRST``      | ouster::sdk::core::ReturnOrder           |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``THERMAL_SHUTDOWN_NORMAL``                 | ``NORMAL``                            | ouster::sdk::core::ThermalShutdownStatus |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``THERMAL_SHUTDOWN_IMMINENT``               | ``IMMINENT``                          | ouster::sdk::core::ThermalShutdownStatus |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``SHOT_LIMITING_NORMAL``                    | ``NORMAL``                            | ouster::sdk::core::ShotLimitingStatus    |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``SHOT_LIMITING_IMMINENT``                  | ``IMMINENT``                          | ouster::sdk::core::ShotLimitingStatus    |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``SHOT_LIMITING_REDUCTION_0_10``            | ``REDUCTION_0_10``                    | ouster::sdk::core::ShotLimitingStatus    |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``SHOT_LIMITING_REDUCTION_10_20``           | ``REDUCTION_10_20``                   | ouster::sdk::core::ShotLimitingStatus    |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``SHOT_LIMITING_REDUCTION_20_30``           | ``REDUCTION_20_30``                   | ouster::sdk::core::ShotLimitingStatus    |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``SHOT_LIMITING_REDUCTION_30_40``           | ``REDUCTION_30_40``                   | ouster::sdk::core::ShotLimitingStatus    |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``SHOT_LIMITING_REDUCTION_40_50``           | ``REDUCTION_40_50``                   | ouster::sdk::core::ShotLimitingStatus    |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``SHOT_LIMITING_REDUCTION_50_60``           | ``REDUCTION_50_60``                   | ouster::sdk::core::ShotLimitingStatus    |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``SHOT_LIMITING_REDUCTION_60_70``           | ``REDUCTION_60_70``                   | ouster::sdk::core::ShotLimitingStatus    |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``SHOT_LIMITING_REDUCTION_70_75``           | ``REDUCTION_70_75``                   | ouster::sdk::core::ShotLimitingStatus    |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``LAYOUT_STANDARD``                         | ``STANDARD``                          | ouster::sdk::osf::ChunksLayout           |
+---------------------------------------------+---------------------------------------+------------------------------------------+
| ``LAYOUT_STREAMING``                        | ``STREAMING``                         | ouster::sdk::osf::ChunksLayout           |
+---------------------------------------------+---------------------------------------+------------------------------------------+


Python Changes
--------------

ouster.sdk.core
^^^^^^^^^^^^^^^

+-----------------------+-----------------------+-----------------+
| Deprecated (0.16)     | Use Instead           | Package         |
+=======================+=======================+=================+
| ``version``           | ``Version``           | ouster.sdk.core |
+-----------------------+-----------------------+-----------------+
| ``lidar_mode``        | ``LidarMode``         | ouster.sdk.core |
+-----------------------+-----------------------+-----------------+
| ``timestamp_mode``    | ``TimestampMode``     | ouster.sdk.core |
+-----------------------+-----------------------+-----------------+
| ``sensor_config``     | ``SensorConfig``      | ouster.sdk.core |
+-----------------------+-----------------------+-----------------+
| ``data_format``       | ``DataFormat``        | ouster.sdk.core |
+-----------------------+-----------------------+-----------------+
| ``SensorCalibration`` | ``CalibrationStatus`` | ouster.sdk.core |
+-----------------------+-----------------------+-----------------+
| ``product_info``      | ``ProductInfo``       | ouster.sdk.core |
+-----------------------+-----------------------+-----------------+
| ``sensor_info``       | ``SensorInfo``        | ouster.sdk.core |
+-----------------------+-----------------------+-----------------+
| ``packet_format``     | ``PacketFormat``      | ouster.sdk.core |
+-----------------------+-----------------------+-----------------+
| ``invalid_version``   | ``INVALID_VERSION``   | ouster.sdk.core |
+-----------------------+-----------------------+-----------------+
| ``min_version``       | ``MIN_VERSION``       | ouster.sdk.core |
+-----------------------+-----------------------+-----------------+

Example Python Migration
++++++++++++++++++++++++

.. code-block:: python

   # Before (deprecated)
   from ouster.sdk.core import lidar_mode, client
   from ouster.sdk.pcap import packet_info, guessed_ports

   # After
   from ouster.sdk.core import LidarMode, Client
   from ouster.sdk.pcap import PacketInfo, GuessedPorts

Example C++ Migration
+++++++++++++++++++++

.. code-block:: cpp

   // Before (deprecated)
   #include "ouster/types.h"
   ouster::sensor::lidar_mode mode = ouster::sensor::lidar_mode::MODE_1024x10;
   ouster::sensor::sensor_config config;

   // After
   #include "ouster/types.h"
   ouster::sdk::core::LidarMode mode = ouster::sdk::core::LidarMode::_1024x10;
   ouster::sdk::core::SensorConfig config;


ScanSource iterators changed to return ``LidarScanSet``
+++++++++++++++++++++++++++++++++++++++++++++++++++++++

No API changes required on the Python side unless you were using type annotations.

API changes required on the cpp side:

.. code:: c++

    /* old code */
    auto source = ouster::open_source(source_file);

    /* new code */
    auto source = ouster::sdk::open_source(source_file);

    /* no changes */
    auto it = source.begin();

    for (auto&& scans : source) {
        do_something(scans[0]);
    }

    /* old code */

    // access through iterator
    std::vector<std::shared_ptr<ouster::LidarScan>>& scans = *it++;

    // for range
    for (std::vector<std::shared_ptr<ouster::LidarScan>>& scans : source) {
        do_something(scans[0]);
    }

    /* new code */

    // access through iterator
    ouster::sdk::core::LidarScanSet scans = *it++;

    // for range
    for (ouster::sdk::core::LidarScanSet scans : source) {
        do_something(scans[0]);
    }

``ChanField`` and ``ChanFieldType`` moved to ``ouster/chanfield.h``.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Developers using C++ will need to add the appropriate include:

.. code:: c++

    #include <ouster/chanfield.h>


``XYZLutT``, ``XYZLut``, and ``make_xyzlut`` moved to ``ouster/xyzlut.h``.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Developers using C++ will need to add the appropriate include:

.. code:: c++

    #include <ouster/xyzlut.h>


Deprecated ``get_field_types(UDPProfileLidar)``
+++++++++++++++++++++++++++++++++++++++++++++++

The ``get_field_types(UDPProfileLidar)`` overload is deprecated. Use the
``get_field_types(SensorInfo)`` overload instead.

.. code-block:: python

   # Before (deprecated)
   from ouster.sdk import core
   fields = core.get_field_types(core.UDPProfileLidar.PROFILE_LIDAR_LEGACY)

   # After
   from ouster.sdk import core
   # sensor_info_path is a pathlib.Path to a sensor info JSON file
   info = core.SensorInfo(sensor_info_path.read_text())
   fields = core.get_field_types(info)


Deprecated ``la_x``, ``la_y``, ``la_z``
++++++++++++++++++++++++++++++++++++++++

Use ``accel()`` instead.

.. code-block:: python

   packet: core.ImuPacket = ...

   # Before (deprecated)
   lx = packet.la_x()
   ly = packet.la_y()
   lz = packet.la_z()

   # After
   lx, ly, lz = packet.accel()


Deprecated ``av_x``, ``av_y``, ``av_z``
++++++++++++++++++++++++++++++++++++++++

Use ``gyro()`` instead.

.. code-block:: python

   packet: core.ImuPacket = ...

   # Before (deprecated)
   ang_x = packet.av_x()
   ang_y = packet.av_y()
   ang_z = packet.av_z()

   # After
   ang_x, ang_y, ang_z = packet.gyro()


Deprecated ``Writer.save(List[LidarScan])``
+++++++++++++++++++++++++++++++++++++++++++

Use ``Writer.save(LidarScanSet)`` instead.

.. code-block:: python

   # Before (deprecated)
   writer.save([lidar_scan1, lidar_scan2])

   # After
   from ouster.sdk import core
   scan_set = core.LidarScanSet([lidar_scan1, lidar_scan2])
   writer.save(scan_set)


Renamed ``data_format::packets_per_frame``
++++++++++++++++++++++++++++++++++++++++++


``data_format::packets_per_frame`` has been renamed to ``lidar_packets_per_frame()`` to match the Python API.

.. code-block:: cpp

   # Before (deprecated)
   auto source = ouster::sdk::pcap::open_source("path/to/file.pcap");
   const auto& info = source->sensor_info()[0];
   std::cout << "packets/frame: " << info.format().packets_per_frame() << "\n";


   # After
   std::cout << "packets/frame: " << info.format().lidar_packets_per_frame() << "\n";


Deprecated point typedefs
++++++++++++++++++++++++++

The old ``PointsT``, ``PointsD``, ``PointsF`` aliases are deprecated in favor of ``PointCloudXYZ*``:

.. code-block:: cpp

   // Before (deprecated)
   ouster::sdk::core::PointsT cloud; 
   ouster::sdk::core::PointsD cloud; 
   ouster::sdk::core::PointsF cloud; 

   // After
   ouster::sdk::core::PointCloudXYZ cloud;
   ouster::sdk::core::PointCloudXYZd cloud;
   ouster::sdk::core::PointCloudXYZf cloud;


Points containers are now row-major
++++++++++++++++++++++++++++++++++++

``ouster::PointsT``, ``ouster::PointsD`` and ``ouster::PointsF`` now use a row-major layout. If you map these buffers
into Eigen matrices, update your ``Eigen::Map`` to use row-major storage:

.. code-block:: cpp

   // Before (column-major map) - default is ColMajor
   Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 3>> mat(points.data(),
                                                                  points.size(), 3);

   // After (row-major map)
   Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>> mat(points.data(),
                                                                                   points.size(), 3);


Updated default format for saved OSF files to ZPNG
++++++++++++++++++++++++++++++++++++++++++++++++++

ZPNG file format cannot be opened by older SDK versions. If you must produce OSF readable by pre 0.16.0 SDK,
override to PNG format with ``--png`` during save.

.. code-block:: bash

   ouster-cli source $SOURCE_FILE_OR_SENSOR save --png $OUTPUT_OSF_FILE


Remove support for connecting to sensors with firmware versions older than 2.4.0.
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

- Upgrade sensor firmware to 2.4.0 or newer before using SDK 0.16.0+.
- If you cannot upgrade firmware yet, remain on SDK 0.15.x for that sensor.

PoseOptimizer API changes
+++++++++++++++++++++++++

Unified constraint API
----------------------

- Removed: ``add_absolute_pose_constraint()``, ``add_pose_to_pose_constraint()``, ``add_point_to_point_constraint()``.
- Use the unified ``add_constraint()`` with constraint objects instead.
- New ``PoseOptimizer`` constructor accepts a constraint JSON file.
- Constraint classes: ``AbsolutePoseConstraint``, ``PoseToPoseConstraint``, ``PointToPointConstraint``.
- Config helpers: ``save_config()``, ``remove_constraint()``, ``get_constraints()``, ``set_constraints()``.

.. code-block:: cpp

   // Before (multiple add_* helpers)
   po.add_absolute_pose_constraint(ts, pose, rotation_weight, translation_weight);

   // After (unified add_constraint)
   ouster::sdk::mapping::AbsolutePoseConstraint c{ts, pose};
   c.rotation_weight = 0.5;
   c.translation_weights = Eigen::Array3d(1.0, 0.5, 0.2);
   po.add_constraint(c);

Per-axis weights
----------------

- Translation weights are now per-axis: pass ``Eigen::Array3d(x, y, z)``.
- Update any code that used a single translation weight to supply a 3-element array.

.. code-block:: cpp

   // Before (single translation weight)
   po.add_pose_to_pose_constraint(ts, rel_pose, rot_w, trans_w);

   // After (per-axis translation weights via constraint object)
   ouster::sdk::mapping::PoseToPoseConstraint c{ts1, ts2, rel_pose};
   c.rotation_weight = rot_w;
   c.translation_weights = Eigen::Array3d(trans_x, trans_y, trans_z);
   po.add_constraint(c);


Removed ``ouster.sdk.core.default_scan_fields``
+++++++++++++++++++++++++++++++++++++++++++++++

Use ``ouster.sdk.util.resolve_field_types`` instead.

.. code-block:: python

   # Before (deprecated)
   fields = core.default_scan_fields(core.LidarMode._1024x10)

   # After
   from ouster.sdk import  open_source
   from ouster.sdk.util import resolve_field_types
   source = open_source("path/to/file.pcap", collate=False)
   fields = resolve_field_types(source.sensor_info)


Removed standalone helpers
+++++++++++++++++++++++++++

Remove deprecated methods: ``core.first_valid_column()``, ``core.last_valid_column()``, ``core.first_valid_column_ts()``, ``core.last_valid_column_ts()``,
``core.first_valid_packet_ts()``, ``core.last_valid_packet_ts()``.

Use the equivalent methods on the ``LidarScan`` class: ``scan.get_first_valid_column()``,
``scan.get_last_valid_column()``, ``scan.get_first_valid_column_timestamp()``, ``scan.get_last_valid_column_timestamp()``, ``scan.get_first_valid_packet_timestamp()``,
and ``scan.get_last_valid_packet_timestamp()``.

.. code-block:: python

   # Before (deprecated)
   first_col = core.first_valid_column(scan)
   last_col = core.last_valid_column(scan)
   first_col_ts = core.first_valid_column_ts(scan)
   last_col_ts = core.last_valid_column_ts(scan)
   first_pkt_ts = core.first_valid_packet_ts(scan)
   last_pkt_ts = core.last_valid_packet_ts(scan)


.. code-block:: python

   # After
   from ouster.sdk import open_source

   src = open_source("path/to/file.pcap")
   for scan_set in src:
       for scan in scan_set:
           if scan is None:
               continue
           first_col = scan.get_first_valid_column()
           last_col = scan.get_last_valid_column()
           first_col_ts = scan.get_first_valid_column_timestamp()
           last_col_ts = scan.get_last_valid_column_timestamp()
           first_pkt_ts = scan.get_first_valid_packet_timestamp()
           last_pkt_ts = scan.get_last_valid_packet_timestamp()
           # process scan...
       break  # remove this break to process all scans
   src.close()


Removed ``firmware_version_from_metadata()``
++++++++++++++++++++++++++++++++++++++++++++

Use ``ouster::sdk::core::SensorInfo::get_version()`` instead.


Previously deprecated removals
+++++++++++++++++++++++++++++++

- Removed ``ScanSource.metadata`` and ``PacketSource.metadata`` (`details <migration-0.14.0-0.15.0.html#updates-to-metadata-retrieval>`_)
- Removed ``ouster.sdk.util.resolve_extrinsics`` (`details <migration-0.14.0-0.15.0.html#deprecated-resolve-extrinsics>`_)
- Removed ``ouster.sdk.sensor.util.build_sensor_config`` (`details <migration-0.14.0-0.15.0.html#deprecated-build-sensor-config>`_)
- Removed ``PointViz::push_frame_buffer_handler()`` and ``PointViz::pop_frame_buffer_handler()`` (`details <migration-0.14.0-0.15.0.html#deprecated-pointviz-frame-buffer-handlers>`_)
- Removed ``ScanSource.sensors_count`` (`details <migration-0.14.0-0.15.0.html#deprecated-sensors-count>`_)
- Removed ``ouster::sensor::parse_config()`` (`details <migration-0.14.0-0.15.0.html#deprecated-parse-config>`_)
- Removed ``ScanBatcher`` constructor (`details <migration-0.14.0-0.15.0.html#deprecated-scanbatcher-constructor>`_)
