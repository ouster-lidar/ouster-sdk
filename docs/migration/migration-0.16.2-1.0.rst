:tocdepth: 3

================================
Migration from 0.16.2 to 1.0.0
================================

The Ouster SDK 1.0.0 release removes APIs that were deprecated in 0.16.x,
reorganizes public C++ headers under module-specific subdirectories, and
introduces several API cleanups across core, sensor, OSF, mapping, PCAP, and
Python bindings.

Prerequisites
+++++++++++++

Migrate to **0.16.0** (or later 0.16.x) before upgrading to 1.0.0. If you have
not yet addressed 0.16.0 namespace and naming changes, start with
:doc:`migration-0.15.0-0.16.0`.

How to find code that needs updating
++++++++++++++++++++++++++++++++++++

1. **Build failures** — removed symbols will produce compile or import errors
   instead of deprecation warnings in prior versions once you upgrade to 1.0.0.
2. **Check the changelog** for the 1.0.0 ``[BREAKING]`` change summary.

   - `CHANGELOG <../reference/changelog.html>`__
3. **Search your codebase** for symbols listed in the sections below.
4. **Check deprecated API pages** in the SDK reference for APIs that still
   work in 1.0 but will be removed later.

   - `C++ <../cpp/api_cpp/page_deprecated.html>`__
   - `Python <../python/api_python/page_deprecated.html>`__
   - Previous migration guides for deprecations introduced before 1.0:
     - :doc:`migration-0.15.0-0.16.0`

Build and include layout
++++++++++++++++++++++++

The ``ouster_client`` library and CMake target are renamed to ``ouster_core``.
Public headers are split into module-scoped paths under ``include/ouster/``, matching the layout already used by ouster/osf/ and ouster/perception/.

There are **no forwarding headers** at the old flat paths. Update every
``#include`` that references the legacy layout.

CMake target rename
^^^^^^^^^^^^^^^^^^^

Link against ``OusterSDK::ouster_core``. A compatibility alias
``OusterSDK::ouster_client`` may still be present but is deprecated and will be
removed in a future release.

``client_version`` metadata string
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Metadata written by the SDK now uses the ``ouster_core`` prefix instead of
``ouster_client`` (for example, ``ouster_core 1.0.0``). Existing metadata
files that record the old string remain readable; only newly written metadata
uses the new value.

Include path changes
^^^^^^^^^^^^^^^^^^^^

Core (``ouster_core``)
~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Before (0.16.x)
     - After (1.0)
   * - ``ouster/types.h``
     - ``ouster/core/types.h``
   * - ``ouster/lidar_scan.h``
     - ``ouster/core/lidar_frame.h``
   * - ``ouster/impl/build.h``
     - ``ouster/core/impl/build.h``
   * - *(other core headers)*
     - ``ouster/core/<name>.h``
   * - *(other core impl)*
     - ``ouster/core/impl/<name>.h``

Sensor (``ouster_sensor``)
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Before (0.16.x)
     - After (1.0)
   * - ``ouster/client.h``
     - ``ouster/sensor/client.h``
   * - ``ouster/sensor_http.h``
     - ``ouster/sensor/sensor_http.h``
   * - ``ouster/sensor_packet_source.h``
     - ``ouster/sensor/sensor_packet_source.h``
   * - ``ouster/sensor_scan_source.h``
     - ``ouster/sensor/sensor_frame_set_source.h``
   * - ``ouster/impl/netcompat.h``
     - ``ouster/sensor/impl/netcompat.h``

PCAP (``ouster_pcap``)
~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Before (0.16.x)
     - After (1.0)
   * - ``ouster/pcap.h``
     - ``ouster/pcap/pcap.h``
   * - ``ouster/os_pcap.h``
     - ``ouster/pcap/os_pcap.h``
   * - ``ouster/pcap_scan_source.h``
     - ``ouster/pcap/pcap_frame_set_source.h``
   * - ``ouster/pcap_packet_source.h``
     - ``ouster/pcap/pcap_packet_source.h``
   * - ``ouster/indexed_pcap_reader.h``
     - ``ouster/pcap/indexed_pcap_reader.h``

Viz (``ouster_viz``)
~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Before (0.16.x)
     - After (1.0)
   * - ``ouster/point_viz.h``
     - ``ouster/viz/point_viz.h``
   * - ``ouster/impl/buffers.h``
     - ``ouster/viz/impl/buffers.h``
   * - ``ouster/impl/zone_monitor_voxel_mesh.h``
     - ``ouster/viz/impl/zone_monitor_voxel_mesh.h``

Mapping (``ouster_mapping``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Before (0.16.x)
     - After (1.0)
   * - ``ouster/slam_engine.h``
     - ``ouster/mapping/slam_engine.h``
   * - ``ouster/pose_optimizer.h``
     - ``ouster/mapping/pose_optimizer.h``
   * - *(other mapping headers)*
     - ``ouster/mapping/<name>.h``
   * - ``ouster/impl/trajectory.h``
     - ``ouster/mapping/impl/trajectory.h``
   * - *(other mapping impl)*
     - ``ouster/mapping/impl/<name>.h``

Algorithm (``ouster_algorithm``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The new ``ouster_algorithm`` package contains shared mapping and perception
algorithms. Link C++ applications that use these APIs against
``OusterSDK::ouster_algorithm``. In Python, import these APIs
from ``ouster.sdk.algorithm``.

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Before (0.16.x)
     - After (1.0)
   * - ``ouster/normals.h``
     - ``ouster/algorithm/normals.h``

OSF and perception headers are unchanged (``ouster/osf/...`` and
``ouster/perception/...``).

Example C++ include migration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: cpp

   // Before (0.16.x)
   #include "ouster/types.h"
   #include "ouster/lidar_scan.h"
   #include "ouster/client.h"
   #include "ouster/pcap_scan_source.h"
   #include "ouster/point_viz.h"
   #include "ouster/slam_engine.h"

   // After (1.0)
   #include "ouster/core/types.h"
   #include "ouster/core/lidar_frame.h"
   #include "ouster/sensor/client.h"
   #include "ouster/pcap/pcap_frame_set_source.h"
   #include "ouster/viz/point_viz.h"
   #include "ouster/mapping/slam_engine.h"

Removed APIs (deprecated in 0.16)
+++++++++++++++++++++++++++++++++

These APIs emitted deprecation warnings in 0.16. They are **removed** in 1.0.0
and must be updated before upgrading.

Removed type and enum aliases
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Duplicate lowercase types and old-style enum entry names from 0.16 are
removed. Use the PascalCase types documented in
:doc:`migration-0.15.0-0.16.0`.

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Removed (0.16.x)
     - Use instead (1.0)
   * - ``sensor_info``
     - ``SensorInfo``
   * - ``lidar_mode``
     - ``LidarMode``
   * - ``timestamp_mode``
     - ``TimestampMode``
   * - ``data_format``
     - ``DataFormat``
   * - ``packet_format``
     - ``PacketFormat``
   * - ``packet_writer``
     - ``PacketFormat``
   * - Old enum constants (e.g. ``MODE_512x10``)
     - New enum constants (e.g. ``_512x10``)

Removed ``Client``, and ``SensorConnection``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In 0.16.x, ``Client`` was a handle returned by ``init_client()``, and ``SensorConnection`` was the Python binding for
the same low-level API. From 1.0.0, use ``SensorPacketSource`` or ``open_packet_source`` to connect to a sensor.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (0.16.x)
         #include "ouster/client.h"
         #include "ouster/packet.h"

         auto client = ouster::sdk::sensor::init_client(
             hostname, udp_dest_host, lidar_mode, timestamp_mode);
         ouster::sdk::core::LidarPacket lidar_packet;
         ouster::sdk::sensor::ClientState state =
             ouster::sdk::sensor::poll_client(*client, 1);
         if (state & ouster::sdk::sensor::ClientState::LIDAR_DATA) {
             ouster::sdk::sensor::read_lidar_packet(*client, lidar_packet);
         }

         // After (1.0)
         #include "ouster/sensor/sensor_packet_source.h"

         ouster::sdk::sensor::SensorPacketSource source(hostname);
         auto event = source.get_packet(1.0);
         if (event.type == ouster::sdk::sensor::ClientEvent::PACKET) {
             auto& packet = event.packet();
         }


   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (0.16.x)
         from ouster.sdk._bindings.client import SensorConnection, ClientState

         conn = SensorConnection(hostname, udp_dest_host, lidar_mode, timestamp_mode)
         state = conn.poll(1)
         if state & ClientState.LIDAR_DATA:
             conn.read_lidar_packet(lidar_packet)

         # After (1.0)
         from contextlib import closing
         from ouster.sdk import open_packet_source

         with closing(open_packet_source(hostname)) as source:
             event = source.get_packet(1.0)
             packet = event.packet()

Removed ``get_field_types(UDPProfileLidar)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use ``get_field_types(SensorInfo)`` instead.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (0.16.x)
         from ouster.sdk import core
         fields = core.get_field_types(core.UDPProfileLidar.LEGACY)

         # After (1.0)
         from ouster.sdk import core
         info = core.SensorInfo(sensor_info_json)
         fields = core.get_field_types(info)


   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (0.16.x)
         auto fields = ouster::sdk::core::get_field_types(
             ouster::sdk::core::UDPProfileLidar::LEGACY);

         // After (1.0)
         ouster::sdk::core::SensorInfo info(metadata_json);
         auto fields = ouster::sdk::core::get_field_types(info);

Removed ``ImuPacket`` legacy accessors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``la_x``, ``la_y``, ``la_z`` and ``av_x``, ``av_y``, ``av_z`` are removed. Use
``accel()`` and ``gyro()`` instead.

.. code-block:: python

   packet: core.ImuPacket = ...

   # Before (0.16.x)
   lx, ly, lz = packet.la_x(), packet.la_y(), packet.la_z()
   gx, gy, gz = packet.av_x(), packet.av_y(), packet.av_z()

   # After (1.0)
   lx, ly, lz = packet.accel()
   gx, gy, gz = packet.gyro()

Removed ``Writer.save(list)`` & ``Writer.save(std::vector)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Pass a ``FrameSet`` instead of a list or vector of frames.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (0.16.x)
         writer.save([scan1, scan2])

         # After (1.0)
         from ouster.sdk import core
         writer.save(core.FrameSet([frame1, frame2]))


   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (0.16.x)
         writer.save({scan1, scan2});

         // After (1.0)
         ouster::sdk::core::FrameSet frame_set({frame1, frame2});
         writer.save(frame_set);

Removed ``to_string(const SensorInfo&)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use ``SensorInfo::to_json_string()`` instead.

.. code-block:: cpp

   // Before (0.16.x)
   std::string s = ouster::sdk::core::to_string(info);

   // After (1.0)
   std::string s = info.to_json_string();

Removed ``default_sensor_info()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use ``SensorInfo::from_default()`` instead.

.. code-block:: cpp

   // Before (0.16.x)
   auto info = ouster::sdk::core::default_sensor_info();

   // After (1.0)
   auto info = ouster::sdk::core::SensorInfo::from_default();

Removed ``get_first_valid_column_timestamp`` and ``get_last_valid_column_timestamp``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

These convenience methods are removed. Index the column timestamp header directly
using ``get_first_valid_column()`` or ``get_last_valid_column()``.

In 1.0, those column index helpers throw when no valid columns are available
(they returned sentinel values or behaved inconsistently in earlier releases).
Wrap calls in exception handling if a frame may have no valid columns.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (0.16.x)
         uint64_t first_col_ts = scan.get_first_valid_column_timestamp();
         uint64_t last_col_ts = scan.get_last_valid_column_timestamp();

         // After (1.0)
         try {
             uint64_t first_col_ts =
                 frame.timestamp()[frame.get_first_valid_column()];
             uint64_t last_col_ts =
                 frame.timestamp()[frame.get_last_valid_column()];
         } catch (const std::runtime_error&) {
             // handle frame with no valid columns
         }


   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (0.16.x)
         first_col_ts = scan.get_first_valid_column_timestamp()
         last_col_ts = scan.get_last_valid_column_timestamp()

         # After (1.0)
         try:
             first_col_ts = frame.timestamp[frame.get_first_valid_column()]
             last_col_ts = frame.timestamp[frame.get_last_valid_column()]
         except RuntimeError:
             # handle frame with no valid columns
             pass

Core API changes
++++++++++++++++

``normals`` moved to ``ouster.sdk.algorithm``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Normal estimation is now part of the shared ``ouster_algorithm`` C++ library.
The C++ function moved from ``ouster::sdk::core`` to
``ouster::sdk::algorithm``. The Python function moved from
``ouster.sdk.core`` to ``ouster.sdk.algorithm``.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      Link against the new CMake target:

      .. code-block:: cmake

         target_link_libraries(my_target PRIVATE OusterSDK::ouster_algorithm)

      Update the header and namespace:

      .. code-block:: cpp

         // Before (0.16.x)
         #include "ouster/normals.h"
         auto result = ouster::sdk::core::normals(
             xyz, range, sensor_origins);

         // After (1.0)
         #include "ouster/algorithm/normals.h"
         auto result = ouster::sdk::algorithm::normals(
             xyz, range, sensor_origins);

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (0.16.x)
         from ouster.sdk.core import normals

         # After (1.0)
         from ouster.sdk.algorithm import normals

Mode string parsing returns optional
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``lidar_mode_of_string()`` and ``timestamp_mode_of_string()`` now return
``std::nullopt`` or ``None`` when the input is not recognized, instead of an
unknown/unspecified value.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (0.16.x)
         auto mode = ouster::sdk::core::lidar_mode_of_string("INVALID_MODE");
         if (mode == ouster::sdk::core::LidarMode::UNSPECIFIED) {
             // handle unrecognized mode string
         }

         // After (1.0)
         auto mode = ouster::sdk::core::lidar_mode_of_string("1024x10");
         if (!mode) {
             // handle unrecognized mode string
         }


   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (0.16.x)
         mode = core.lidar_mode_of_string("INVALID_MODE")
         if mode == core.LidarMode.UNSPECIFIED:
             # handle unrecognized mode string

         # After (1.0)
         mode = core.lidar_mode_of_string("1024x10")
         if mode is None:
             # handle unrecognized mode string

``LidarMode`` is now a struct
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``LidarMode`` changed from an enum class to a struct to support arbitrary
sensor modes. 

``UNSPECIFIED`` values are also removed from ``LidarMode``,
``TimestampMode`` and ``OperatingMode``. Check for parse failures with
``std::nullopt`` or ``None`` instead of comparing against ``UNSPECIFIED``.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (0.16.x) — LidarMode enum
         ouster::sdk::core::LidarMode mode =
             ouster::sdk::core::LidarMode::MODE_1024x10;

         // After (1.0) — LidarMode struct
         ouster::sdk::core::LidarMode mode =
             ouster::sdk::core::LidarMode::_1024x10;
         uint32_t cols = mode.columns;
         unsigned int fps = mode.fps;

         // Before (0.16.x) — UNSPECIFIED
         auto ts_mode = ouster::sdk::core::timestamp_mode_of_string("INVALID");
         if (ts_mode == ouster::sdk::core::TimestampMode::UNSPECIFIED) {
         }
         auto op_mode = ouster::sdk::core::operating_mode_of_string("INVALID");
         if (op_mode == ouster::sdk::core::OperatingMode::UNSPECIFIED) {
         }

         // After (1.0) — optional return values
         ts_mode = ouster::sdk::core::timestamp_mode_of_string("TIME_FROM_PTP_1588");
         if (!ts_mode) {
         }
         op_mode = ouster::sdk::core::operating_mode_of_string("NORMAL");
         if (!op_mode) {
         }


   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (0.16.x) — LidarMode enum
         mode = core.LidarMode.MODE_1024x10

         # After (1.0) — LidarMode struct
         mode = core.LidarMode._1024x10
         cols = mode.columns
         fps = mode.fps

         # Before (0.16.x) — UNSPECIFIED sentinels
         ts_mode = core.timestamp_mode_of_string("INVALID")
         if ts_mode == core.TimestampMode.UNSPECIFIED:
             pass
         op_mode = core.operating_mode_of_string("INVALID")
         if op_mode == core.OperatingMode.UNSPECIFIED:
             pass

         # After (1.0) — None on parse failure
         ts_mode = core.timestamp_mode_of_string("TIME_FROM_PTP_1588")
         if ts_mode is None:
             pass
         op_mode = core.operating_mode_of_string("NORMAL")
         if op_mode is None:
             pass

``LidarFrame`` constructor changes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

C++ constructors now take **height then width** (matching Python). Some
constructors require an explicit ``columns_per_packet`` argument for REV8
sensors.

.. code-block:: cpp

   // Before (0.16.x) — width, height order
   ouster::sdk::core::LidarScan scan(w, h, field_types);

   // After (1.0) — height, width order with columns_per_packet
   ouster::sdk::core::LidarFrame frame(h, w, field_types, columns_per_packet);

``LidarFrame`` packet timestamp methods
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Packet timestamp helpers on ``LidarFrame`` were refactored for clearer
per-stream behavior and stricter error handling.

**New overloads:** Each of the four methods now accepts an optional
``PacketType`` / ``stream`` argument to query lidar, IMU, or zone timestamps
independently.

**Behavior change:** ``get_first_valid_packet_timestamp()``,
``get_last_valid_packet_timestamp()``, ``get_min_valid_packet_timestamp()``, and
``get_max_valid_packet_timestamp()`` (and their ``PacketType`` overloads) now
throw when no valid packets are available instead of returning ``0``. In Python,
this surfaces as ``RuntimeError``.

**Deprecated:** ``get_first_valid_lidar_packet_timestamp()`` and
``get_last_valid_lidar_packet_timestamp()`` remain available in 1.0 but emit
deprecation warnings. They still return ``0`` when no valid lidar packets are
found. Use the ``PacketType::Lidar`` / ``PacketType.Lidar`` overloads instead.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         using ouster::sdk::core::LidarFrame;
         using ouster::sdk::core::PacketType;

         // Before (0.16.x) — returned 0 when unavailable
         uint64_t first_ts = scan.get_first_valid_packet_timestamp();
         if (first_ts == 0) {
             // handle missing timestamps
         }

         uint64_t lidar_first = scan.get_first_valid_lidar_packet_timestamp();

         // After (1.0) — throws when unavailable
         try {
             uint64_t first_ts = frame.get_first_valid_packet_timestamp();
             uint64_t lidar_first =
                 frame.get_first_valid_packet_timestamp(PacketType::Lidar);
             uint64_t imu_min =
                 frame.get_min_valid_packet_timestamp(PacketType::Imu);
             uint64_t zone_max =
                 frame.get_max_valid_packet_timestamp(PacketType::Zone);
         } catch (const std::runtime_error&) {
             // handle missing timestamps
         }


   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         from ouster.sdk._bindings.client import PacketType

         # Before (0.16.x) — returned 0 when unavailable
         first_ts = scan.get_first_valid_packet_timestamp()
         if first_ts == 0:
             # handle missing timestamps
             pass

         lidar_first = scan.get_first_valid_lidar_packet_timestamp()

         # After (1.0) — raises RuntimeError when unavailable
         try:
             first_ts = frame.get_first_valid_packet_timestamp()
             lidar_first = frame.get_first_valid_packet_timestamp(PacketType.Lidar)
             imu_min = frame.get_min_valid_packet_timestamp(PacketType.Imu)
             zone_max = frame.get_max_valid_packet_timestamp(PacketType.Zone)
         except RuntimeError:
             # handle missing timestamps
             pass

``FrameSet::valid_frames()`` reference type
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

C++ ``valid_frames()`` now yields ``std::shared_ptr<LidarFrame>&`` instead of
``LidarScan&``.

.. code-block:: cpp

   // Before (0.16.x)
   ouster::sdk::core::LidarScanSet scan_set = ...;
   for (auto& scan : scan_set.valid_scans()) {
       scan.complete();  // LidarScan&
   }

   // After (1.0)
   ouster::sdk::core::FrameSet frame_set = ...;
   for (auto& frame_ptr : frame_set.valid_frames()) {
       frame_ptr->complete();  // shared_ptr<LidarFrame>&
   }

``FieldInfo`` renamed to ``FieldDecodeInfo``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (0.16.x)
         ouster::sdk::core::FieldInfo field;

         // After (1.0)
         ouster::sdk::core::FieldDecodeInfo field;


   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (0.16.x)
         from ouster.sdk.core import FieldInfo

         # After (1.0)
         from ouster.sdk.core import FieldDecodeInfo

``PacketWriter`` merged into ``PacketFormat``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use ``PacketFormat`` for packet serialization. Helpers such as
``frame_to_packets()`` take ``PacketFormat`` instead of ``PacketWriter``.
``scan_to_packets()`` remains as a deprecated alias.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (0.16.x)
         from ouster.sdk.core import PacketWriter
         writer = PacketWriter.from_metadata(metadata)
         packets = scan_to_packets(scan, writer)

         # After (1.0)
         from ouster.sdk.core import PacketFormat, frame_to_packets
         pf = PacketFormat.from_info(sensor_info)
         packets = frame_to_packets(frame, pf)


   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (0.16.x)
         ouster::sdk::core::PacketWriter writer(format);

         // After (1.0)
         auto format = std::make_shared<ouster::sdk::core::PacketFormat>(info);
         ouster::sdk::core::LidarPacket packet(format);

``thermal_shutdown`` and ``shot_limiting`` return enums
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

These methods now return ``ThermalShutdownStatus`` and
``ShotLimitingStatus`` instead of ``uint8_t``.

.. code-block:: cpp

   // Before (0.16.x)
   uint8_t status = packet.thermal_shutdown();
   if (status == static_cast<uint8_t>(ouster::sdk::core::ThermalShutdownStatus::IMMINENT)) {
      // ...
   }

   // After (1.0)
   auto status = packet.thermal_shutdown();
   if (status == ouster::sdk::core::ThermalShutdownStatus::IMMINENT) {
       // ...
   }

``XYZLut`` and Cartesian projection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``make_xyz_lut`` and ``cartesianT`` moved to impl. Use ``XYZLut`` directly or
the cached ``SensorInfo::xyzlut()`` accessors.

Cached ``xyzlut()`` accessors always include extrinsics; the
``use_extrinsics`` argument is removed. To omit extrinsics, construct
``XYZLut`` explicitly.

``XYZLut`` direction, offset, ``h``, and ``w`` are now ``const`` and cannot be
mutated after construction. Construct a new ``XYZLut`` instead of modifying an
existing one.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (0.16.x)
         lut = info.xyzlut(use_extrinsics=False)
         lut.direction[:] = 0.0  # in-place mutation was possible

         # After (1.0) — cached accessors always include extrinsics
         lut = info.xyzlut()
         # lut.direction[:] = 0.0  # ValueError: assignment to read-only array
         # lut.h = 128               # AttributeError: read-only property
         lut_no_ext = core.XYZLut(info, use_extrinsics=False)


   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (0.16.x)
         ouster::sdk::core::XYZLut lut(info);
         lut.direction = other_direction;  // OK
         lut.h = 128;                    // OK

         // After (1.0) — preferred cached accessor
         auto lut = info.xyzlut();
         // lut.direction = other_direction;  // compile error: direction is const
         // lut.h = 128;                      // compile error: h is const
         ouster::sdk::core::XYZLut lut_no_ext(info, false);

Row-major pose and extrinsic matrices
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Pose and extrinsic matrix storage switched to row-major (C-style) ordering
across core, mapping, OSF, and viz. When passing NumPy arrays
to SDK functions,
use ``order='C'``. When mapping to Eigen, use row-major storage.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         import numpy as np
         # Ensure row-major layout when constructing matrices for the SDK
         extrinsics = np.array(matrix_4x4, dtype=np.float64, order='C')


   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // When mapping point buffers, use row-major Eigen maps
         Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>
             points(map_ptr, rows, 3);

``FrameBatcher`` move-assignment removed
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Move-construct ``FrameBatcher`` instances; move-assignment is deleted.

.. code-block:: cpp

   ouster::sdk::core::FrameBatcher batcher1(info);
   ouster::sdk::core::FrameBatcher batcher2(std::move(batcher1));  // OK
   // batcher1 = std::move(batcher2);  // compile error in 1.0

``Object`` moved to core namespace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Before (0.16.x)
   from ouster.sdk.zone_monitor import Object

   # After (1.0)
   from ouster.sdk.core import Object

Removed ``Triangle::inside()`` (C++)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This helper is no longer part of the public C++ API. Implement point-in-triangle
tests in application code if needed.

Sensor API changes
++++++++++++++++++

``get_config`` and ``set_config`` return types
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

C++ return types now match the Python dict-like API surface.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (0.16.x)
         ouster::sdk::core::SensorConfig config;
         if (!ouster::sdk::sensor::get_config(hostname, config, true)) {
             throw std::runtime_error("Error getting sensor config.");
         }
         if (!ouster::sdk::sensor::set_config(hostname, config)) {
             throw std::runtime_error("Error setting sensor config.");
         }

         // After (1.0)
         auto config = ouster::sdk::sensor::get_config(hostname, true);
         ouster::sdk::sensor::set_config(hostname, config);

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         from ouster.sdk import sensor
         config = sensor.get_config(hostname)   # dict-like
         sensor.set_config(hostname, config)

``SensorHttp`` import path (Python)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``SensorHttp`` is no longer re-exported from ``ouster.sdk.core``.

.. code-block:: python

   # Before (0.16.x)
   from ouster.sdk.core import SensorHttp

   # After (1.0)
   from ouster.sdk.sensor import SensorHttp

OSF API changes
+++++++++++++++

``Writer.save`` requires timestamps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``Writer::save(FrameSet)`` and ``AsyncWriter::save(FrameSet)`` throw
if any frame in the set has missing timestamps. Ensure timestamps are populated
before saving.

OSF frame ordering
^^^^^^^^^^^^^^^^^^

OSF files now index and order frames by the **greatest** valid packet timestamp
rather than the first. Replay and indexing code that assumed first-packet
ordering may observe different frame sequences for partial frames.

``slice_with_cast`` renamed to ``slice_and_cast``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (0.16.x)
         from ouster.sdk import osf
         sliced = osf.slice_with_cast(scan, field_types)

         # After (1.0)
         sliced = osf.slice_and_cast(frame, field_types)


   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (0.16.x)
         auto sliced = ouster::sdk::osf::slice_with_cast(scan, field_types);

         // After (1.0)
         auto sliced = ouster::sdk::osf::slice_and_cast(frame, field_types);

OSF Writer ``sensor_info`` indexed accessors removed
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use ``sensor_info()`` to get the full collection, then index normally.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (0.16.x)
         info = writer.sensor_info(0)
         count = writer.sensor_info_count()

         # After (1.0)
         infos = writer.sensor_info()
         info = infos[0]
         count = len(infos)


   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (0.16.x)
         auto info = writer.sensor_info(0);
         auto count = writer.sensor_info_count();

         // After (1.0)
         auto infos = writer.sensor_info();
         auto& info = infos[0];
         auto count = infos.size();

SLAM and mapping changes
++++++++++++++++++++++++

SLAM and localization engine refactor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``SlamBackend``, ``LocalizationBackend``, and ``kind`` string selection at
``create()`` time are removed. Use ``SlamEngine::create()`` and
``LocalizationEngine::create()`` with ``LIOSlamConfig`` or
``LIOLocalizationConfig``. The backend identifier changed from
``"kiss"`` to ``"lio"`` in the current SDK.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (0.16.x)
         #include <ouster/mapping/slam_engine.h>
         ouster::sdk::mapping::SlamConfig config;
         config.backend = "kiss";
         ouster::sdk::mapping::SlamEngine slam(source.sensor_info(), config);

         // After (1.0)
         #include <ouster/core/open_source.h>
         #include <ouster/mapping/slam_engine.h>

         auto source = ouster::sdk::open_source(source_file);
         ouster::sdk::mapping::LIOSlamConfig config;
         auto slam = ouster::sdk::mapping::SlamEngine::create(
             source.sensor_info(), config);


   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (0.16.x)
         from ouster.sdk.mapping import SlamEngine, SlamConfig
         config = SlamConfig()
         config.backend = "kiss"
         slam = SlamEngine(source.sensor_info, config)

         # After (1.0)
         from ouster.sdk import open_source
         from ouster.sdk.mapping import SlamEngine, SlamConfig

         source = open_source(source_file)
         config = SlamConfig.create("lio")
         slam = SlamEngine.create(source.sensor_info, config)

Pose optimizer trajectory alignment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``PoseOptimizer::initialize_trajectory_alignment()`` now returns the applied
4x4 transform matrix instead of ``bool``.

.. code-block:: cpp

   // Before (0.16.x)
   bool ok = optimizer.initialize_trajectory_alignment();

   // After (1.0)
   Eigen::Matrix4d transform = optimizer.initialize_trajectory_alignment();

Pose optimizer point-constraint row/column indices
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``AbsolutePointConstraint`` and ``PointToPointConstraint`` still expose ``row``,
``col``, ``row1``, and ``col1``, but those indices now refer to the
**staggered** ``LidarFrame`` layout instead of **destaggered** image coordinates.

In 0.16.x, Pose Optimizer converted destaggered column indices internally
using ``pixel_shift_by_row``. In 1.0, pass indices that match
``frame.field(ChanField.RANGE)[row, col]`` directly. The same rule applies to
JSON constraints (``ABSOLUTE_POINT``, ``POINT_TO_POINT``).

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         from ouster.sdk import mapping

         # Before (0.16.x) — row/col picked from a destaggered 2D image
         constraint = mapping.AbsolutePointConstraint(
             timestamp=ts,
             row=27,
             col=1894,  # destaggered column
             return_idx=1,
             absolute_position=[40.0, 30.0, 10.0],
         )
         optimizer.add_constraint(constraint)

         # After (1.0) — row/col from staggered LidarFrame fields
         constraint = mapping.AbsolutePointConstraint(
             timestamp=ts,
             row=27,
             col=100,  # matches frame.field(ChanField.RANGE)[row, col]
             return_idx=1,
             absolute_position=[40.0, 30.0, 10.0],
         )
         optimizer.add_constraint(constraint)

         # If you still have destaggered coordinates, convert before use:
         W = frame.w
         shift = info.format.pixel_shift_by_row[row] % W
         col_staggered = (col_destaggered + shift) % W


   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (0.16.x) — row/col from destaggered image coordinates
         ouster::sdk::mapping::AbsolutePointConstraint constraint(
             ts, 27, 1894, 1, position);

         // After (1.0) — row/col from staggered LidarFrame coordinates
         ouster::sdk::mapping::AbsolutePointConstraint constraint(
             ts, 27, 100, 1, position);

         // If you still have destaggered coordinates, convert before use:
         const int W = static_cast<int>(frame.w);
         const int shift = (info.format.pixel_shift_by_row[row] % W + W) % W;
         const int col_staggered = (col_destaggered + shift) % W;

CLI: ``--auto-constraints`` renamed to ``--auto-gps``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   # Before (0.16.x)
   ouster-cli source <source> pose_optimize --auto-constraints

   # After (1.0)
   ouster-cli source <source> pose_optimize --auto-gps

Indexed PCAP changes
++++++++++++++++++++

``IndexedPcapReader`` API cleanup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``frame_id_rolled_over()`` is removed. ``current_frame_id()`` now returns
``std::optional<uint32_t>`` instead of ``std::optional<uint16_t>``.

.. code-block:: cpp

   // Before (0.16.x)
   if (reader.frame_id_rolled_over()) { ... }
   std::optional<uint16_t> fid = reader.current_frame_id();

   // After (1.0)
   std::optional<uint32_t> fid = reader.current_frame_id();

Python-specific changes
+++++++++++++++++++++++

Disallow implicit ndarray conversions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Python bindings no longer perform implicit ndarray dtype or memory-order
conversions. Pass arrays with the expected ``dtype`` and ``order``.

.. code-block:: python

   # Before (0.16.x) — implicit conversion accepted
   arr = np.asarray(data)  # e.g., float64
   scan.add_field("CUSTOM", arr)

   # After (1.0) — pass explicitly typed, row-major arrays
   import numpy as np
   arr = np.asarray(data, dtype=np.float32, order='C')
   frame.add_field("CUSTOM", arr)

``FrameSet.valid_frames()`` and ``valid_indices()`` return lists
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

These methods return ``list`` instead of iterators. Code that mutated the
collection while iterating may need adjustment — take a snapshot first.

.. code-block:: python

   # After (1.0) — returns a list
   for frame in frame_set.valid_frames():
       process(frame)

``ouster.sdk.zone_monitor`` moved to ``ouster.sdk.core``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Update imports for zone monitor types (except ``Object``, which also moved to
core — see above).

.. code-block:: python

   # Before (0.16.x)
   from ouster.sdk.zone_monitor import Zone, ZoneSet

   # After (1.0)
   from ouster.sdk.core import Zone, ZoneSet

Viz utility changes
^^^^^^^^^^^^^^^^^^^

* ``Cloud.set_key_rgb`` and ``Cloud.set_key_rgba`` are removed. Use
  ``Cloud.set_key`` instead.
* ``AutoExposure`` and ``BeamUniformityCorrector`` moved from
  ``ouster.sdk.core._utils`` to ``ouster.sdk.core``.

.. code-block:: python

   # Before (0.16.x)
   from ouster.sdk.viz import Cloud
   from ouster.sdk.core._utils import AutoExposure
   cloud = Cloud(num_points)
   cloud.set_key_rgb(key_array)

   # After (1.0)
   from ouster.sdk.viz import Cloud
   from ouster.sdk.core import AutoExposure
   cloud = Cloud(num_points)
   cloud.set_key(key_array)

* ``push_point_viz_handler`` and ``BoundMethod`` are removed. Use push functions
  on ``PointViz`` directly.
  
.. code-block:: python

   # Before (0.16.x) — inside SimpleViz
   from ouster.sdk.viz import push_point_viz_handler

   viz = PointViz("Example Viz")
   myviz = MyViz(viz)
   push_point_viz_handler(viz, myviz, MyViz._handle_key)

   # After (1.0)
   from ouster.sdk.viz import PointViz

   viz = PointViz("Example Viz")
   myviz = MyViz(viz)
   viz.push_key_handler(myviz._handle_key)

CLI changes
+++++++++++

* ``ouster-cli util benchmark`` and ``ouster-cli util benchmark-sensor`` were removed in 1.0.0.

Recommended updates (deprecated in 1.0.0, will be removed in upcoming releases)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

These APIs still work in 1.0.0 but emit deprecation warnings on usage. Update before
the next major release. See the deprecated API lists for the full set:

- `C++ Deprecated List <../cpp/api_cpp/page_deprecated.html>`__
- `Python Deprecated List <../python/api_python/page_deprecated.html>`__

Deprecated Duplicative PCAP Readers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``ouster::sdk::pcap::PcapReader``, ``ouster::sdk::pcap::IndexedPcapReader`` and ``ouster::sdk::pcap::PlaybackHandle`` were deprecated.
Use ``ouster::sdk::pcap::PcapPacketSource`` instead.

.. code-block:: cpp

   // Initialize PcapPacketSource
   ouster::sdk::pcap::PcapPacketSource src(pcap_filename);

   // Read out packets from each sensor
   for (auto& iter: src) {
       auto& sensor_idx = iter.first;
       auto& packet = iter.second;
   }

Deprecated ``ouster::sdk::pcap::PcapWriter``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use ``ouster::sdk::pcap::RecordHandle`` instead.

.. code-block:: cpp

   // Initialize writer
   std::shared_ptr<ouster::sdk::pcap::RecordHandle> handle =
       ouster::sdk::pcap::record_initialize(out_filename, 1430);

   // Write packets
   ouster::sdk::pcap::record_packet(*handle, src_ip, dst_ip, src_port,
                                    dst_port, buf, buffer_size, ts_us);

   // Finalize
   ouster::sdk::pcap::record_uninitialize(*handle);

Deprecated ``core::cartesian()`` 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use ``XYZLut`` call syntax instead.

.. code-block:: cpp

   // Initialize a lookup table once from sensor metadata
   ouster::sdk::core::XYZLut lut(sensor_info);

   // Deprecated in 1.0
   auto pts = ouster::sdk::core::cartesian(scan, lut);

   // Recommended
   auto pts = lut(frame);

``n_cols_of_lidar_mode`` or ``frequency_of_lidar_mode``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use ``LidarMode.columns`` and ``LidarMode.fps``.

.. code-block:: cpp

   // Deprecated
   auto cols = ouster::sdk::core::n_cols_of_lidar_mode(mode);

   // Recommended
   auto cols = mode.columns;

``add_custom_profile`` old overload
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The overload that takes an explicit profile number is deprecated. Use the
overload that allocates a new enum value.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Deprecated
         add_custom_profile(profile_nr, name, fields, chan_data_size);

         // Recommended
         auto profile = add_custom_profile(name, fields, chan_data_size);


   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Deprecated
         add_custom_profile(profile_nr, name, fields, chan_data_size)

         # Recommended
         profile_nr = add_custom_profile(name, fields, chan_data_size)

Insufficient ``LidarFrame`` constructors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Constructors that do not provide enough information are deprecated. In C++,
this includes ``LidarFrame(h, w)``, ``LidarFrame(DataFormat)``,
``LidarFrame(h, w, profile, columns_per_packet)``, and ``LidarFrame(sensor_info)``
by value. Python deprecates only the first two dimension-only overloads;
``LidarFrame(sensor_info)`` is not deprecated there because the binding already
takes a shared ``SensorInfo``.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Deprecated
         // LidarScan is deprecated; use LidarFrame instead
         // Below methods for LidarFrame are also deprecated
         LidarFrame scan(h, w);
         LidarFrame scan(format);
         LidarFrame scan(h, w, profile, columns_per_packet);
         LidarFrame scan(sensor_info);

         // Recommended
         LidarFrame frame(h, w, field_types, columns_per_packet);
         LidarFrame frame(std::make_shared<SensorInfo>(sensor_info));


   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Deprecated
         # LidarScan is deprecated; use LidarFrame instead
         # Below methods for LidarFrame are also deprecated
         scan = core.LidarFrame(h, w)
         scan = core.LidarFrame(h, w, profile, columns_per_packet)

         # Recommended
         frame = core.LidarFrame(h, w, field_types, columns_per_packet)
         frame = core.LidarFrame(sensor_info)

``LidarFrame::pose()`` renamed to ``body_to_world()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``LidarFrame::pose()`` is now accessed via ``LidarFrame::body_to_world()`` for
better clarity. Previous ``pose`` accessor remains available but emits
deprecation warnings.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (0.16.x)
         auto& poses = scan.pose();
         auto column_pose = scan.get_column_pose(col);

         // After (1.0)
         auto& poses = frame.body_to_world();
         auto column_pose = frame.get_column_pose(col);


   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (0.16.x)
         poses = scan.pose
         column_pose = scan.pose[col]

         # After (1.0)
         poses = frame.body_to_world
         column_pose = frame.body_to_world[col]

``get_first_valid_lidar_packet_timestamp`` and ``get_last_valid_lidar_packet_timestamp``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

These lidar-only helpers are deprecated. Use the ``PacketType`` overloads on the
general packet timestamp methods instead. Unlike the replacements, the deprecated
methods still return ``0`` when no valid lidar packets are available.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Deprecated
         uint64_t first = scan.get_first_valid_lidar_packet_timestamp();
         uint64_t last = scan.get_last_valid_lidar_packet_timestamp();

         // Recommended
         uint64_t first =
             frame.get_first_valid_packet_timestamp(PacketType::Lidar);
         uint64_t last =
             frame.get_last_valid_packet_timestamp(PacketType::Lidar);


   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         from ouster.sdk._bindings.client import PacketType

         # Deprecated
         first = scan.get_first_valid_lidar_packet_timestamp()
         last = scan.get_last_valid_lidar_packet_timestamp()

         # Recommended
         first = frame.get_first_valid_packet_timestamp(PacketType.Lidar)
         last = frame.get_last_valid_packet_timestamp(PacketType.Lidar)

``SensorInfo::extrinsic`` renamed to ``sensor_to_body``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``SensorInfo::extrinsic`` is now accessed via ``SensorInfo::sensor_to_body``.

On the python side, ``SensorInfo.extrinsic`` remains available but emits
deprecation warning. On the C++ side, it is a breaking change.

Metadata JSON still stores this matrix under ``ouster-sdk.extrinsic``
for backward compatibility with existing files.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (0.16.x)
         info.extrinsic = transform;
         auto lut = ouster::sdk::core::XYZLut(info, true);

         // After (1.0)
         info.sensor_to_body = transform;
         auto lut = ouster::sdk::core::XYZLut(info, true);


   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (0.16.x)
         info.extrinsic = transform
         lut = core.XYZLut(info, use_extrinsics=True)

         # After (1.0)
         info.sensor_to_body = transform
         lut = core.XYZLut(info, use_extrinsics=True)

``PacketFormat.from_metadata``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Deprecated
   pf = PacketFormat.from_metadata(json_string)

   # Recommended
   pf = PacketFormat.from_info(sensor_info)

Size-only packet constructors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Default and size-only constructors for ``LidarPacket``, ``ImuPacket``, and
``ZonePacket`` are deprecated. Construct packets from a ``PacketFormat`` instead.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Deprecated
         ouster::sdk::core::LidarPacket lidar_packet;
         ouster::sdk::core::LidarPacket lidar_packet(65536);
         ouster::sdk::core::ImuPacket imu_packet;
         ouster::sdk::core::ImuPacket imu_packet(65536);
         ouster::sdk::core::ZonePacket zone_packet;
         ouster::sdk::core::ZonePacket zone_packet(65536);

         // Recommended
         auto pf = std::make_shared<ouster::sdk::core::PacketFormat>(info);
         ouster::sdk::core::LidarPacket lidar_packet(pf);
         ouster::sdk::core::ImuPacket imu_packet(pf);
         ouster::sdk::core::ZonePacket zone_packet(pf);


   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Deprecated
         lidar_packet = core.LidarPacket(65536)
         imu_packet = core.ImuPacket(65536)
         zone_packet = core.ZonePacket(65536)

         # Recommended
         pf = core.PacketFormat.from_info(sensor_info)
         lidar_packet = core.LidarPacket(pf)
         imu_packet = core.ImuPacket(pf)
         zone_packet = core.ZonePacket(pf)

``FrameBatcher`` call syntax
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         from ouster.sdk.core import (
             SensorInfo, LidarScan, LidarFrame, ScanBatcher, FrameBatcher)

         info = SensorInfo(...)

         # Before (0.16.x)
         batcher = ScanBatcher(info)
         scan = None
         for idx, packet in packet_source:
             if not scan:
                 scan = LidarScan(info)
             if batcher(packet, scan):
                 yield scan
                 scan = None

         # After (1.0)
         batcher = FrameBatcher(info)
         frame = None
         for idx, packet in packet_source:
             if not frame:
                 frame = LidarFrame(info)
             if batcher.batch(packet, frame):
                 yield frame
                 frame = None


   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         auto info = ...;  // std::shared_ptr<ouster::sdk::core::SensorInfo>

         // Before (0.16.x)
         ouster::sdk::core::ScanBatcher batcher(*info);
         ouster::sdk::core::LidarScan scan{info};
         if (batcher(lidar_packet, scan)) {
             // scan complete
         }

         // After (1.0)
         ouster::sdk::core::FrameBatcher frame_batcher(*info);
         ouster::sdk::core::LidarFrame frame{info};
         if (frame_batcher.batch(lidar_packet, frame)) {
             // frame complete
         }

CLI ``metadata`` subcommand
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   # Deprecated
   ouster-cli source <source> metadata

   # Recommended
   ouster-cli source <source> sensor_info

Scan-to-Frame API renames (1.0)
+++++++++++++++++++++++++++++++

The 1.0 release renames all remaining public ``scan``-named identifiers to
``frame`` to align with the ``LidarFrame`` / ``FrameSet`` terminology introduced
in 0.16.  Deprecated type aliases remain available for now and will be removed in
a future release.  Direct method and property renames are breaking immediately.

Type aliases
^^^^^^^^^^^^

These deprecated type aliases are still available but will be removed in a future
release.  Update your code to use the new names.

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Deprecated (1.0)
     - Replacement
   * - ``LidarScan``
     - ``LidarFrame``
   * - ``LidarScanSet``
     - ``FrameSet``
   * - ``ScanSource``
     - ``FrameSetSource``
   * - ``ScanSourceOptions``
     - ``FrameSetSourceOptions``
   * - ``ScanSourceMetadata``
     - ``FrameSetSourceMetadata``
   * - ``ScanBatcher``
     - ``FrameBatcher``
   * - ``ScanIterator``
     - ``FrameSetIterator``
   * - ``ScanSourceWrapper``
     - ``FrameSetSourceWrapper``
   * - ``AnyScanSource``
     - ``AnyFrameSetSource``
   * - ``MultiScanSource``
     - ``MultiFrameSetSource``
   * - ``PcapScanSource``
     - ``PcapFrameSetSource``
   * - ``OsfScanSource``
     - ``OsfFrameSetSource``
   * - ``SensorScanSource``
     - ``SensorFrameSetSource``

Deprecated forwarding headers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following headers are deprecated and will be removed in a future release.
Include the renamed headers instead.

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Deprecated header
     - New header
   * - ``ouster/core/lidar_scan.h``
     - ``ouster/core/lidar_frame.h``
   * - ``ouster/core/lidar_scan_set.h``
     - ``ouster/core/frame_set.h``
   * - ``ouster/core/scan_source.h``
     - ``ouster/core/frame_set_source.h``
   * - ``ouster/core/scan_source_utils.h``
     - ``ouster/core/frame_set_source_utils.h``
   * - ``ouster/osf/lidarscan_encoder.h``
     - ``ouster/osf/lidarframe_encoder.h``
   * - ``ouster/osf/png_lidarscan_encoder.h``
     - ``ouster/osf/png_lidarframe_encoder.h``
   * - ``ouster/osf/zpng_lidarscan_encoder.h``
     - ``ouster/osf/zpng_lidarframe_encoder.h``
   * - ``ouster/osf/stream_lidar_scan.h``
     - ``ouster/osf/stream_lidar_frame.h``
   * - ``ouster/osf/osf_scan_source.h``
     - ``ouster/osf/osf_frame_set_source.h``
   * - ``ouster/pcap/pcap_scan_source.h``
     - ``ouster/pcap/pcap_frame_set_source.h``
   * - ``ouster/sensor/sensor_scan_source.h``
     - ``ouster/sensor/sensor_frame_set_source.h``

``FieldClass::SCAN_FIELD`` renamed to ``FRAME_FIELD``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``FieldClass::SCAN_FIELD`` is deprecated.  A deprecated alias remains available;
update usages to ``FieldClass::FRAME_FIELD``.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (1.0)
         frame.add_field("FLAGS", data, ouster::sdk::core::FieldClass::SCAN_FIELD);

         // After (1.0)
         frame.add_field("FLAGS", data, ouster::sdk::core::FieldClass::FRAME_FIELD);

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (1.0)
         frame.add_field("FLAGS", data, core.FieldClass.SCAN_FIELD)

         # After (1.0)
         frame.add_field("FLAGS", data, core.FieldClass.FRAME_FIELD)

``FrameSet::scans()`` and ``valid_scans()`` renamed
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``FrameSet::scans()`` is renamed to ``frames()`` and ``valid_scans()`` is
renamed to ``valid_frames()``.  Deprecated aliases are provided; update
usages to use the new names before the last supported version.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         ouster::sdk::core::FrameSet frame_set = ...;

         // Before (1.0)
         for (auto& f : frame_set.scans()) { ... }
         for (auto& f : frame_set.valid_scans()) { ... }

         // After (1.0)
         for (auto& f : frame_set.frames()) { ... }
         for (auto& f : frame_set.valid_frames()) { ... }

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (1.0)
         for f in frame_set.scans(): ...
         for f in frame_set.valid_scans(): ...

         # After (1.0)
         for f in frame_set.frames(): ...
         for f in frame_set.valid_frames(): ...

``SensorFrameSetSource`` method renames
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``dropped_scans()`` and ``get_scan()`` are renamed to ``dropped_frames()``
and ``get_frame()``.  Deprecated aliases are provided; update usages before
the last supported version.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (1.0)
         auto dropped = source.dropped_scans();
         auto frame = source.get_scan();

         // After (1.0)
         auto dropped = source.dropped_frames();
         auto frame = source.get_frame();

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (1.0)
         dropped = source.dropped_scans
         frame = source.get_scan()

         # After (1.0)
         dropped = source.dropped_frames
         frame = source.get_frame()

Python keyword argument renames
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Several Python APIs renamed their ``scan``/``scans`` keyword arguments to
``frame``/``frames``.

.. code-block:: python

   from ouster.sdk import core, osf

   # FrameSet constructor
   # Before (1.0)
   frame_set = core.FrameSet(scans=[frame1, frame2])
   # After (1.0)
   frame_set = core.FrameSet(frames=[frame1, frame2])

   # Writer.save / AsyncWriter.save
   # Before (1.0)
   writer.save(scan=frame)
   writer.save(scans=frame_set)
   # After (1.0)
   writer.save(frame=frame)
   writer.save(frames=frame_set)

   # align_clouds
   # Before (1.0)
   result = align_clouds(source_scan=src, target_scan=tgt)
   # After (1.0)
   result = align_clouds(source_frame=src, target_frame=tgt)

Python viz property renames
^^^^^^^^^^^^^^^^^^^^^^^^^^^

``scan_num`` and ``scans_per_sec`` are renamed to ``frame_num`` and
``frames_per_sec``. Deprecated aliases are provided; update usages before
the last supported version.

.. code-block:: python

   # Before (1.0)
   print(viz.scan_num)
   print(viz.scans_per_sec)

   # After (1.0)
   print(viz.frame_num)
   print(viz.frames_per_sec)

``ls_show`` renamed to ``lf_show``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``ls_show`` viz helper function has been renamed to ``lf_show`` to align
with the ``LidarFrame`` terminology. The old name is deprecated and will be
removed in a future release.

.. code-block:: python

   # Before (1.0)
   from ouster.sdk.viz import ls_show
   ls_show(frames)

   # After (1.1)
   from ouster.sdk.viz import lf_show
   lf_show(frames)

Deprecated Python module paths
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following Python modules were renamed as part of the ``LidarFrame``
migration. Deprecated shim modules remain at the old paths and will be removed
in a future release. Update any direct module-path imports to the new names.

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Deprecated module path
     - New module path
   * - ``ouster.sdk.core.scan_ops``
     - ``ouster.sdk.core.frame_ops``
   * - ``ouster.sdk.core.clipped_scan_source``
     - ``ouster.sdk.core.clipped_frame_set_source``
   * - ``ouster.sdk.core.masked_scan_source``
     - ``ouster.sdk.core.masked_frame_set_source``
   * - ``ouster.sdk.core.reduced_scan_source``
     - ``ouster.sdk.core.reduced_frame_set_source``
   * - ``ouster.sdk.bag.bag_scan_source``
     - ``ouster.sdk.bag.bag_frame_set_source``
   * - ``ouster.sdk.viz.scans_accumulator``
     - ``ouster.sdk.viz.frames_accumulator``
   * - ``ouster.sdk.examples.lidar_scan``
     - ``ouster.sdk.examples.lidar_frame``

.. code-block:: python

   # Before (1.0)
   import ouster.sdk.core.scan_ops as so
   from ouster.sdk.bag.bag_scan_source import BagFrameSetSource

   # After (1.1)
   import ouster.sdk.core.frame_ops as so
   from ouster.sdk.bag.bag_frame_set_source import BagFrameSetSource
