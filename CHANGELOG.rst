=========
Changelog
=========


[1.0.0]
=========
* Support newer ``click`` versions (>=8.2) in the Python SDK CLI and relax the ``click`` requirement to ``>=8.1.3, <9``.
* Refactor map export to downsample points as they get added into the map, flushing the pointcloud data to disk as the map accumulates.
* Added ``ouster::sdk::core::Vector3iHash``, a non-``std`` voxel-index hash with a splitmix64 finalizer that produces good 64-bit dispersion.
* [CLI/BREAKING] Dropped ``--max-z`` and ``--min-z`` options from ``ouster-cli source ... save (.pcd/.ply)``
  and ``ouster-cli source ... save``; use ``filter Z`` to crop points along the Z axis before saving.
* [BREAKING] Change ``lidar_mode_of_string()`` and ``timestamp_mode_of_string()`` to return an optional instead of unknown/unspecified if the enum was not found.
* [BREAKING] Switch pose and extrinsic matrix storage to row-major/C-style ordering across core, mapping, viz, OSF, Python bindings, and related array/serialization helpers.
* Use Nanobind instead of Pybind for bindings.
* [BREAKING] Remove duplicate deprecated enums following old naming conventions in favor of their new namesakes.
* [BREAKING] Remove deprecated ``ImuPacket`` accessors such as ``la_x`` and ``av_x`` in favor of new accessors such as ``accel()`` and ``gyro()``.
* [BREAKING] Remove deprecated ``osf::Writer.save(List[LidarScan])`` and C++ analogues in favor of ``osf::Writer.save(LidarScanSet)``.
* [BREAKING] Remove deprecated duplicate definitions for classes and enums renamed in 0.16 such as ``sensor_info`` in favor of their new namesakes.
* [BREAKING] Remove deprecated ``Client`` (C++) and ``SensorConnection`` (C++) in favor of ``SensorPacketSource``.
* [BREAKING] Remove deprecated ``get_field_types(UDPProfileLidar)`` method in favor of ``get_field_types(SensorInfo)``.
* [BREAKING] Moved ``ouster::sdk::core::make_xyz_lut`` to impl. Use ``ouster::sdk::core::XYZLut`` instead.
* Add cached ``SensorInfo::xyzlut()`` accessors in C++ and Python (``xyzlut_float``/``xyzlut_double``).
* [BREAKING] Cached ``SensorInfo`` LUT accessors always include extrinsics; remove the ``use_extrinsics`` argument from ``SensorInfo::xyzlut()`` and ``xyzlut_float``/``xyzlut_double``. To omit extrinsics, construct ``XYZLut`` with ``use_extrinsics=false`` (C++) or ``use_extrinsics=False`` (Python).
* [BREAKING] Moved ``ouster.sdk.core._utils.AutoExposure`` and ``ouster.sdk.core._utils.BeamUniformityCorrector`` to ``ouster.sdk.core``.
* [BREAKING] Removed ``ouster.sdk.core.io_types``; import ``OusterIoType``, ``io_type``, ``io_type_from_extension``, and ``extension_from_io_type`` from ``ouster.sdk.core`` instead.
* [BREAKING] Changed return types of ``ouster::sdk::sensor::get_config`` and ``set_config`` to match Python.
* [BREAKING] Remove ``SensorHttp`` re-export from ``ouster.sdk.core``; import from ``ouster.sdk.sensor`` instead.
* Add Python binding for ``SensorHttp.get_firmware_version_string()``.
* [BREAKING] Moved ``ouster::sdk::core::cartesianT()`` to impl.
* [BREAKING] Removed ``ouster.sdk.viz.util.push_point_viz_handler`` and ``ouster.sdk.viz.BoundMethod``. Their functionality is no longer necessary and the ``push`` functions on ``PointViz`` can just be used directly.
* [FUTURE BREAKING] Deprecated ``ouster::sdk::core::cartesian()`` in favor of ``XYZLut operator()``.
* [BREAKING] Change ``ouster::sdk::core::LidarScan`` constructors in C++ to take height then width rather than the opposite to match Python.
* Mark ``LidarScan`` move construction and move assignment ``noexcept`` in C++.
* Allow constructing ``ouster::sdk::viz::Cloud`` from an ``XYZLut``.
* [BREAKING] Rename ``--auto-constraints`` to ``--auto-gps`` in ``ouster-cli source ... pose_optimize``.
* Add ``--auto-loop`` and ``PoseOptimizer::add_relative_loop_constraints()`` for loop-closure generation.
* Improve pose optimizer viz with constraint selection/removal and pose-to-pose cloud highlights.
* Add ``align_clouds()`` for scan/point-cloud alignment, including ``LidarScanSet`` multi-sensor alignment in C++ and Python (``ouster.sdk.algorithm``), and ``ouster-cli source ... align``.
* Add the ``ouster_algorithm`` C++ package for shared mapping and perception algorithms, including point-cloud alignment, normal estimation, ground segmentation, and averaging voxel downsampling.
* Improve ``ouster-cli source ... plumb``: averaged IMU acceleration is now rotated into the body frame, and the computed correction is composed with the existing sensor extrinsics instead of replacing them.
* [BREAKING] Move C++ ``normals`` from ``ouster::sdk::core`` (header ``ouster/normals.h``) to ``ouster::sdk::algorithm`` (header ``ouster/algorithm/normals.h``); Python ``normals`` is now available from ``ouster.sdk.algorithm`` instead of ``ouster.sdk.core``.
* [FUTURE BREAKING] Deprecated the Python compatibility import ``ouster.sdk.perception.normals`` in favor of ``ouster.sdk.algorithm.normals``.
* Add ``ouster::sdk::core::Pose`` and ``ouster.sdk.core.Pose``, a translation-plus-quaternion pose type with matrix and Euler conversions in C++/Python.
* Add ``GroundSegEngine`` in C++/Python and ``ouster-cli source ... ground`` to annotate scans with a ``GROUND`` mask.
  Ground segmentation uses SLAM scan poses when present, otherwise uses sensor extrinsics; pre-plumb sources with IMU data before
  segmentation for gravity-aligned results.
* [BREAKING] Change ``PoseOptimizer.initialize_trajectory_alignment()`` to return the applied 4x4 transform instead of ``bool``.
* [BREAKING] Pose Optimizer point-constraint row/column ids now use staggered ids instead of destaggered.
* [BREAKING] Changed ``ouster::sdk::core::LidarMode`` to a struct from an enum to support arbitrary sensor lidar modes without requiring additional changes.
* [BREAKING] Removed now unused ``UNSPECIFIED`` enum values.
* [BREAKING] Made members of ``ouster::sdk::core::XYZLut`` const.
* Greatly reduced memory use of visualizer especially in accumulating use cases.
* [BUGFIX] Handle the case when slam is passed scans with invalid poses leading to problems downstream in the slam pipeline.
* [BREAKING] ``ouster::sdk::osf::Writer::save(const LidarScanSet&)`` and ``ouster::sdk::osf::AsyncWriter::save(const LidarScanSet&)`` will now throw an exception on receiving scans with missing timestamps.
* [FUTURE BREAKING] Deprecated ``ouster-cli source <source> metadata`` in favor of ``ouster-cli source <source> sensor_info`` to match naming in the rest of the SDK.
* [BUGFIX] Fix csv export to filter non PIXEL_FIELDs, handle multi sensor save and handle 3D PIXEL_FIELDs.
* Updated LidarScan collation methodology to have improved results with synchronized sensors when autodetected.
* No longer save collations via the CLI in OSF unless they were in the original source or added via the detect command.
* [BREAKING] OSF files now index/order scans by the greatest valid packet timestamp rather than the first valid packet timestamp. This better replicates the behavior of PCAP or Sensor scan sources especially with partial scans.
* [BREAKING] Disallow implicit ndarray type and ordering conversions for Python bindings to prevent accidental mismatches and performance degradation.
* [BREAKING] Removed redundant Python ``ouster.sdk.viz.Cloud.set_key_rgb`` and ``ouster.sdk.viz.Cloud.set_key_rgba`` functions. Use ``ouster.sdk.viz.Cloud.set_key`` instead.
* [BUGFIX] No longer read past the end of legacy IMU packets while parsing to avoid possible crashes.
* Add FOV cycling in the visualizer: ``CTRL+-`` / ``CTRL+=`` cycle through field-of-view levels (30°–150° in 15° steps, default 90°) in perspective mode.
* Add ``--resolution`` option to ``ouster-cli source <source> viz`` to control the initial window resolution. Accepts ``<width>x<height>`` (e.g. ``1920x1080``) or a named resolution (e.g. ``1080p``, ``2k``, ``4k``). Ignored when ``--maximize`` or ``--fullscreen`` is set.
* Add support for directly visualizing RGB8 images in PointViz.
* [BREAKING] Change ``ouster::sdk::core::LidarScanSet::valid_scans()`` to yield ``std::shared_ptr<LidarScan>&`` instead of ``LidarScan&`` to match normal iteration and resolve accidental copies occurring in the Python bindings.
* [BREAKING] Change Python ``LidarScanSet.valid_scans()`` and ``LidarScanSet.valid_indices()`` to return lists instead of iterators.
* Add SLAM local map visualization to ``ouster-cli source slam viz`` with ``--viz-local-map``.
* [FUTURE BREAKING] Deprecate ``ouster::sdk::core::n_cols_of_lidar_mode()`` in favor of ``LidarMode.columns``.
* [FUTURE BREAKING] Deprecate ``ouster::sdk::core::frequency_of_lidar_mode()`` in favor of ``LidarMode.fps``.
* [BREAKING] Remove deprecated ``ouster::sdk::core::to_string(const SensorInfo&)``; use ``SensorInfo::to_json_string()`` instead.
* [BREAKING] Rename ``ouster::sdk::core::default_sensor_info()`` to ``ouster::sdk::core::SensorInfo::from_default()`` to match Python.
* [FUTURE BREAKING] Add new ``ouster::sdk::core::add_custom_profile()`` method which allocates and returns its own enum value for the new profile and deprecated the old method.
* [FUTURE BREAKING] Deprecate ``ouster::sdk::core::LidarScan`` constructors that provided insufficient information such as ``LidarScan(h, w)``, ``LidarScan(DataFormat)``, ``LidarScan(h, w, profile, columns_per_packet)``, and the ``LidarScan(sensor_info)`` constructor that does not take in a shared pointer.
* [BREAKING] Require providing ``columns_per_packet`` argument on some ``ouster::sdk::core::LidarScan`` constructors to avoid issues with REV8 sensors.
* [BREAKING] Rename ``ouster::sdk::osf::slice_with_cast`` to ``ouster::sdk::osf::slice_and_cast`` to match Python.
* Add ``--max-iterations`` option to ``ouster-cli source slam`` and ``ouster-cli source localize`` to set the maximum number of ICP registration iterations.
* Make ``ouster-cli source <source> sensor_replay`` able to be chained.
* Add ``ouster-cli source --lidar-profile <UDPProfileLidar>`` option to spoof the profile of a source to allow converting it to another profile.
* [BUGFIX] Fix ``ouster::sdk::core::destagger`` implementation not working with Eigen::Tensor.
* [BUGFIX] Fix parsing of shot_limiting in packet headers.
* [BUGFIX] Fix collation accidentally waiting for an additional scan before yielding. This could lead to up to an extra frame of latency with SensorScanSources.
* [BREAKING] ``ouster::sdk::core::PacketFormat::thermal_shutdown()`` and ``ouster::sdk::core::PacketFormat::shot_limiting()`` as well as ``ouster::sdk::core::Packet::thermal_shutdown()`` and ``ouster::sdk::core::Packet::shot_limiting()`` now return their respective enum types rather than ``uint8_t``.
* [BREAKING] Removed indexed/count ``sensor_info`` accessors from OSF Writer in C++ (``ouster::sdk::osf::Writer::sensor_info(int)``, ``ouster::sdk::osf::Writer::sensor_info_count()``) and Python (``writer.sensor_info(stream_index)``, ``writer.sensor_info_count()``). Use ``sensor_info()`` to get the full collection, then use normal indexing and length operations (``operator[]``/``size()`` in C++, ``[]``/``len()`` in Python).
* [BREAKING] Merge ``PacketWriter`` into ``PacketFormat`` and remove the separate ``PacketWriter`` Python/C++ API surface. Use ``PacketFormat`` for packet serialization (e.g. ``PacketFormat.from_info()``); helpers such as ``scan_to_packets()`` now take ``PacketFormat`` instead of ``PacketWriter``.
* [BREAKING] Move ``ouster.sdk.zone_monitor`` API surface to ``ouster.sdk.core``.
* [BREAKING] Rename ``FieldInfo`` to ``FieldDecodeInfo`` in C++ and Python.
* [BREAKING] Refactor SLAM/localization APIs: remove ``SlamBackend`` and ``LocalizationBackend``. Removed selecting engine implementations with a ``kind`` string at ``create()`` time (e.g. ``"kiss"``); instead use ``SlamEngine::create()`` and ``LocalizationEngine::create()`` with ``LIOSlamConfig``/``LIOLocalizationConfig``. The supported backend identifier changed from ``"kiss"`` to ``"lio"`` for ``SlamConfig.create()``/``LocalizationConfig.create()`` in Python.
* [BREAKING] Remove unused ``Triangle::inside()`` from the public C++ API.
* [BREAKING] Remove dead ``ouster-cli util benchmark`` and ``ouster-cli util benchmark-sensor`` CLI commands.
* [BREAKING] Clean up indexed PCAP frame-id APIs by removing ``IndexedPcapReader::frame_id_rolled_over()`` and changing ``current_frame_id()`` to return ``optional<uint32_t>`` instead of ``optional<uint16_t>``.
* [BREAKING] Rename core library layout from ``ouster_client`` to ``ouster_core``.
* [BREAKING] Change metadata ``client_version`` strings written by the SDK from the ``ouster_client`` prefix to ``ouster_core`` (for example, ``ouster_core 1.0.0``). Existing metadata files that record the old string remain readable; only newly written metadata uses the new value.
* [BREAKING] Standardize SDK code layout and public include paths by splitting headers into module-scoped namespaces such as ``ouster/core``, ``ouster/sensor``, ``ouster/pcap``, ``ouster/mapping`` and ``ouster/viz``.
* [FUTURE BREAKING] Deprecate ``PacketFormat.from_metadata()`` in favor of ``PacketFormat.from_info()``.
* [FUTURE BREAKING] Deprecate default/size-only packet constructors for ``LidarPacket``, ``ImuPacket``, and ``ZonePacket`` in favor of constructors that take ``PacketFormat``.
* [FUTURE BREAKING] Deprecate ``ScanBatcher`` call syntax (``operator()`` / ``__call__()``) in favor of ``ScanBatcher.batch()``.
* [BREAKING] Delete ``ScanBatcher`` move-assignment; use move construction instead.
* [BUGFIX] Improve viz stability and correctness (GL buffer cleanup, GIF save crash/deadlock handling, AOI fixes, palette/type mismatches, and race-condition fixes when callbacks and rendering overlap).
* [BUGFIX] Add a guard for ``None`` scans before printing shot-limiting warnings in viz paths.
* [BUGFIX] Improve CLI and processing robustness (``filter``/``clip``/``mask`` correctness with non-pixel fields, ``LidarScan.__str__`` fixes, unsupported chan-field handling in OSF read, and safer error handling in save/CRC paths).
* [BUGFIX] Improve SLAM reliability and determinism across multi-sensor and sparse-data edge cases, including IMU availability and deskew behavior.
* [BUGFIX] Fix null-pointer dereferences in OSF scan source iteration when message decoding fails.
* [BUGFIX] Improve recovery of OSF files truncated by a few bytes at the end of the file.
* [BUGFIX] Fix ``reduce`` dropping ``columns_per_packet`` from ``sensor_info``, which could crash downstream processing.
* [BUGFIX] Fix ``--initial-pose`` so it is used again for SLAM and localization.
* Add viz workflow improvements including 3D point selection, object overlays/selection, IMU-gap rendering, non-blocking GIF export, Ctrl+S to toggle sky visualization in SimpleViz, and a per-column ``TIMESTAMP`` point-cloud coloring mode.
* Add ``--clear-on-loop`` to ``ouster-cli source <source> viz`` (and ``SimpleViz(clear_on_loop=...)``) to clear accumulated history and trajectories when a looping source restarts; default is to preserve history across loops.
* Make ``RGB`` the default cloud coloring mode for viz accumulation when present; otherwise fall back to ``REFLECTIVITY`` as per legacy behavior.
* Show thermal shutdown and shot-limiting warnings in the viz window instead of printing them to the console.
* Improve viz UI scaling on macOS: text now follows the actual window content scale rather than a hardcoded 2x scale, the OSD scale also applies to the right-side OSD, and a 0.5x UI scale step is available.
* Add ``MouseEventType`` and a ``mode`` argument to ``PointViz.push_mouse_button_handler``/``push_mouse_pos_handler`` in C++ and Python so mouse callbacks can specify which buttons they handle.
* Add zone emulation and detection workflow improvements, including additional emulation options and richer object outputs.
* ``ouster-cli source ... stats`` now reports object totals for ``LidarScanSet``-level and per-scan ``objects``.
* ``ouster-cli source ... stats`` now counts lidar, IMU, and zone packets and reports missing packet counts for incomplete frames.
* Make ``SensorInfo.beam_azimuth_angles``, ``beam_altitude_angles``, and ``DataFormat.pixel_shift_by_row`` mutable NumPy arrays in Python bindings.
* Add support for opening Ouster Studio Web links (including share links) in the CLI and via Python ``ouster.sdk.open_source()``.
* Add support for downloading Ouster Studio Web links in the CLI via ``ouster-cli source <data-app-url> download <filename>``.
* ``ouster-cli source <url> dump`` and ``info`` now work directly on web-hosted OSF files; add ``osf::Reader::size()`` (``Reader.size`` in Python).
* [FUTURE BREAKING] Deprecate ``LidarScan::pose()`` accessor; it has been renamed to ``LidarFrame::body_to_world()``.
* [BREAKING] C++ side ``SensorInfo::extrinsic`` renamed to ``SensorInfo::sensor_to_body``.
* [FUTURE BREAKING] Python side ``SensorInfo.extrinsic`` is deprecated. Access via ``SensorInfo.sensor_to_body`` instead.
* [BREAKING] This release drops support for macOS 11.x, 12.x and 13.x.
* Ouster SDK now supports Python 3.14 and wheels are available on PyPI.
* [FUTURE BREAKING] Deprecated ``ls_show`` viz function; use ``lf_show`` instead.
* [FUTURE BREAKING] Deprecated ``LidarScan`` type alias; use ``LidarFrame`` directly. ``LidarScanSet`` is deprecated in favor of ``FrameSet``. ``ScanSource``, ``ScanSourceOptions``, and ``ScanSourceMetadata`` are deprecated in favor of ``FrameSetSource``, ``FrameSetSourceOptions``, and ``FrameSetSourceMetadata``. ``ScanBatcher`` and ``ScanIterator`` are deprecated in favor of ``FrameBatcher`` and ``FrameSetIterator``. ``ScanSourceWrapper``, ``AnyScanSource``, and ``MultiScanSource`` are deprecated in favor of ``FrameSetSourceWrapper``, ``AnyFrameSetSource``, and ``MultiFrameSetSource``. ``PcapScanSource``, ``OsfScanSource``, and ``SensorScanSource`` are deprecated in favor of their ``FrameSetSource`` counterparts.
* [FUTURE BREAKING] Deprecated forwarding headers ``ouster/core/lidar_scan.h``, ``ouster/core/lidar_scan_set.h``, ``ouster/core/scan_source.h``, ``ouster/core/scan_source_utils.h``, ``ouster/osf/lidarscan_encoder.h``, ``ouster/osf/png_lidarscan_encoder.h``, ``ouster/osf/zpng_lidarscan_encoder.h``, ``ouster/osf/stream_lidar_scan.h``, ``ouster/osf/osf_scan_source.h``, ``ouster/pcap/pcap_scan_source.h``, and ``ouster/sensor/sensor_scan_source.h``; include the renamed headers directly.
* [FUTURE BREAKING] Deprecated Python module paths ``ouster.sdk.core.scan_ops``, ``ouster.sdk.core.clipped_scan_source``, ``ouster.sdk.core.masked_scan_source``, ``ouster.sdk.core.reduced_scan_source``, ``ouster.sdk.bag.bag_scan_source``, ``ouster.sdk.viz.scans_accumulator``, and ``ouster.sdk.examples.lidar_scan``; import from the renamed modules instead (e.g. ``ouster.sdk.core.frame_ops``).
* [FUTURE BREAKING] Deprecated ``FieldClass::SCAN_FIELD``; use ``FieldClass::FRAME_FIELD`` instead.
* [FUTURE BREAKING] Renamed ``FrameSet::scans()`` to ``FrameSet::frames()`` and ``FrameSet::valid_scans()`` to ``FrameSet::valid_frames()``. Deprecated aliases are provided for both.
* [FUTURE BREAKING] Renamed ``SensorFrameSetSource::dropped_scans()`` to ``dropped_frames()`` and ``get_scan()`` to ``get_frame()``. Deprecated aliases are provided for both.
* [BREAKING] Renamed ``PcapPacketSource::begin_scan()`` to ``begin_frame()``.
* [BREAKING] Renamed Python ``FrameSet.__init__`` keyword argument ``scans`` to ``frames``; renamed ``Writer.save()`` and ``AsyncWriter.save()`` keyword arguments ``scan`` to ``frame`` and ``scans`` to ``frames``; renamed ``align_clouds()`` keyword arguments ``source_scan`` and ``target_scan`` to ``source_frame`` and ``target_frame``.
* [FUTURE BREAKING] Renamed Python viz properties ``scan_num`` to ``frame_num`` and ``scans_per_sec`` to ``frames_per_sec``. Deprecated aliases are provided for both.
* [BREAKING] ``LidarFrame::get_first_valid_column_timestamp`` and ``LidarFrame::get_last_valid_column_timestamp`` methods have been removed. Use ``frame.timestamp()[frame.get_first_valid_column()]`` and ``frame.timestamp()[frame.get_last_valid_column()]`` pattern instead.
* [FUTURE BREAKING] ``LidarFrame::get_first_valid_lidar_packet_timestamp`` and ``LidarFrame::get_last_valid_lidar_packet_timestamp`` methods have been deprecated. Use ``LidarFrame::get_first_valid_packet_timestamp(PacketType::Lidar)`` and ``LidarFrame::get_last_valid_packet_timestamp(PacketType::Lidar)`` instead.
* [BREAKING] ``LidarFrame::get_first_valid_column`` and ``LidarFrame::get_last_valid_column`` will now throw if no valid columns are available.
* [BREAKING] ``LidarFrame::get_first_valid_packet_timestamp`` and ``LidarFrame::get_last_valid_packet_timestamp`` will now throw if no valid packet timestamps are available.
* Port frame_ops to C++ and add a new ``select`` operation to pick specific beams from a scan.
* Add ``ouster-cli source <source> save_extrinsics <filename>`` command to save extrinsics to a json file.
* [BUGFIX] Fixed windows header issue in cmake files
* Change ``ouster-cli source <source> sensor_replay`` command to support either Hypercorn, Waitress or Flask.
* [FUTURE BREAKING] Deprecated ``ouster::sdk::pcap::PcapReader`` and ``ouster::sdk::pcap::IndexedPcapReader``. Use ``ouster::sdk::pcap::PcapPacketSource`` instead.
* [FUTURE BREAKING] Deprecated ``ouster::sdk::pcap::PcapWriter``. Use ``ouster::sdk::pcap::RecordHandle`` instead.
* [FUTURE BREAKING] Deprecated ``ouster::sdk::pcap::PlaybackHandle``. Use ``ouster::sdk::pcap::PcapPacketSource`` instead.
* Add support for LAS/LAZ file reading and writing with RGB in ``ouster-cli source <source> viz`` and ``ouster-cli source <source> save``.
* Update the ``voxel_downsample`` method to support different strategies ``FIRST_N_POINT``, ``AVERAGE_POINT``, and ``RANDOM`` with ``RANDOM`` being the default strategy.
* Add support for ``ouster-cli source <source> save --downsample <voxel_size> --strategy <strategy>`` command to downsample the point cloud.


[0.16.2]
========
* Add support for RGB lidar packet profiles ``RNG19_RFL8_SIG16_NIR16_RGB16`` and ``RNG19_RFL8_SIG16_NIR16_RGB16_DUAL``, including new chan fields ``R``, ``G``, ``B``, ``RGB`` and ``ChanFieldType::FLOAT16`` support.
* Add HDR RGB visualization path for float16 RGB fields in viz, update default image/cloud mode selection to prefer RGB when present.
* Add color-aware PLY/PCD loading for map visualization, so ``ouster-cli source <pointcloud> viz`` now displays colored point clouds.
* Improve zone emulation workflow: add ``--keep-live-ids`` and ``--keep-sensor-to-body`` to ``emulate_zones``, add ``--extrinsics zone`` to source extrinsics handling, and fix zone mesh transform usage for BODY-frame rendering.
* Apply autoexposure to exported float16 RGB point cloud data during mapping point cloud conversion to produce properly normalized RGB output.
* [BREAKING] Python image processing correctors now use ``update()`` instead of being called directly.
* [BREAKING] Removed ``core.Scans`` from the Python API. Use ``ouster::sdk::open_source`` instead.

[0.16.1]
=========

* [BUGFIX] Add ``--coord-frame`` (``SENSOR|BODY|WORLD``) to ``ouster-cli source ... filter`` for XYZ filtering. ``WORLD`` now applies ``dewarp(points, scan.pose)`` before thresholding, ``BODY`` remains the default for backward compatibility, and ``SENSOR`` uses XYZ without extrinsics.
* [BUGFIX] Fix crash in ``LidarScan.__str__`` when scans contain non-pixel fields (IMU, GNSS, ZONE, or CHAR fields). The ``to_string()`` function now properly handles these field types, with special formatting for CHAR fields to display NMEA sentences.
* [BUGFIX] Avoid divide-by-zero in IMU visualization when IMU magnitudes are zero.
* [BUGFIX] Fix AOI picker showing normalized values for unknown fields in the viz.
* [BREAKING] Renamed ``-F`` flag to ``-f`` in ``ouster-cli source`` command (short for ``--filter`` flag), which drops scans with missing data.
* [BUGFIX] PLY maker minor change (e.g. support for no valid key field, ``--field NONE``).
* [BUGFIX] Use stderr instead of stdout for logging in ouster-cli.
* [BUGFIX] Fix ``filter``, ``clip``, and ``mask`` bugs: correct filter_xyz semantics (points inside bounds zeroed), restrict operations to PIXEL_FIELD only to avoid crashes with IMU/GNSS fields, and use RANGE2 coordinates for second-return fields.
* [BUGFIX] Handle unsupported ChanFieldTypes when reading OSF files: skip unsupported fields with a warning instead of crashing (forward compatibility with newer OSF files).
* Add ``--legacy`` flag for OSF export (PNG compression + drop CHAR/ZONE_STATE for SDK 0.12–0.15 compatibility), separate from ``--png`` (PNG compression only).
* [BUGFIX] Update TUM format in trajectory saving as per official specification.
* [BUGFIX] Fix viz GL buffer cleanup leaks: add missing ``glDeleteBuffers()`` for GLCuboid, GLRings, and GLImage to prevent GPU memory growth on repeated create/destroy.
* [BUGFIX] Fix ValueError in viz when resizing before scans received.
* [BUGFIX] Fix crash when saving a GIF when frame duration is negative (e.g. source has looped); fix viz deadlock when saving image recording.
* [BUGFIX] Fix improper handling of sensors with lidar data disabled.
* [BUGFIX] SLAM crashes when at least one sensor doesn't have imu data in a multi-sensor setup with the Synchronous IMU feature enabled.
* [BUGFIX] SLAM uses un-corrected timestamps when using the the inertial integration deskew method with multi-sensor unsynchronized setups.
* Pose Optimizer: constraint ID counter moved to constraint base class; constraint IDs are now logged when adding/processing constraints, and reassign constraint id when conflict instead of throwing.
* [BUGFIX] Remove the skipping on single scans in SLAM and localize pipelines for multi-sensor datasets.
* [BUGFIX] PNGs from the viz now embed the Display P3 profile so their colors match the viz window.
* Add ``--http-addr`` option to ``sensor_replay`` plugin (e.g. for use with Ouster Detect).
* Update README with community forum and support info.
* Normalize license files for GitHub detection.

Important Notes
---------------

* This will be the last release that supports macOS 11.x, 12.x and 13.x.


[0.16.0]
=========

* [BUGFIX] Fix issue where reflectivity data is being reported as zero when range data is zero in AOI labels in the viz.
* [BUGFIX] Avoid a crash on the viz while screen recording using SHIFT-X
* Clear AOI selection in the viz when right click is pressed and there is a preexisting selection.
* Extend AOI selection to show 3 axis fields properly when right click in viz images.
* Add support for low bandwidth dual returns lidar profile (PROFILE_RNG15_RFL8_NIR8_DUAL).
* [BREAKING] Renamed ``data_format::packets_per_frame()`` to ``lidar_packets_per_frame()``
* Add support for saving and reading new ZPNG based OSF files which are several times faster and have better compression ratios.
* [BREAKING] Make ZPNG the default format for saved OSF files. This format cannot be opened by older SDK versions.
* [BREAKING] Change ``ouster-cli source <osf> dump`` JSON output to return the OSF version as a semver-style string instead of an integer.
* Add --png option to ``save`` in ``ouster-cli`` to save legacy PNG based OSF files.
* Add support for storing IMU data in ``LidarScan`` for the ``ACCEL32_GYRO32_NMEA`` profile via new ChanFields: ``IMU_ACC``, ``IMU_GYRO``, ``IMU_TIMESTAMP``, ``IMU_MEASUREMENT_ID``, ``IMU_STATUS``, ``IMU_PACKET_TIMESTAMP``, ``IMU_ALERT_FLAGS``, ``POSITION_STRING``, ``POSITION_LAT_LONG``, and ``POSITION_TIMESTAMP``.
* Add support to ``SensorHttp`` for setting gateway addresses through ``SensorHttp::set_static_ip(static_ip, gateway_address)``.
* Add ``--set-gateway`` to ouster-cli ``network`` command for setting sensor's gateway addresses.
* [BREAKING] Change ``ouster::PointsT``, ``ouster::PointsD`` and ``ouster::PointsF`` to Row Major layout to improve performance
* [BREAKING] Remove deprecated ``ScanSource.metadata`` and ``PacketSource.metadata`` in favor of ``*.sensor_info``.
* [BREAKING] Remove deprecated ``ouster.sdk.util.resolve_extrinsics`` for explicitly passing extrinsic file names to sources.
* [BREAKING] Remove deprecated ``ouster.sdk.sensor.util.build_sensor_config`` for the same functionality already included in SensorScanSource/SensorPacketSource.
* [BREAKING] Remove deprecated ``PointViz::push_frame_buffer_handler()`` and ``PointViz::pop_frame_buffer_handler()``.
  in C++ and Python in favor the new screenshot facilities ``get_screenshot(), save_screenshot(), and toggle_screen_recording()``.
* [BREAKING] Remove deprecated ``ScanSource.sensors_count``. Use length of ``ScanSource.sensor_info`` instead.
* [BREAKING] Remove deprecated ``core.first_valid_column()``, ``core.last_valid_column()``, ``core.first_valid_column_ts()``, ``core.last_valid_column_ts()``,
  ``core.first_valid_packet_ts()``, ``core.last_valid_packet_ts()`` in favor of their replacements on the ``LidarScan`` class: ``scan.get_first_valid_column()``,
  ``scan.get_last_valid_column()``, ``scan.get_first_valid_column_timestamp()``, ``scan.get_last_valid_column_timestamp()``, ``scan.get_first_valid_packet_timestamp()``,
  and ``scan.get_last_valid_packet_timestamp()``.
* [BREAKING] Remove deprecated ``ouster::sensor::parse_config()`` in favor of ``ouster::parse_and_validate_config()``.
* [BREAKING] Remove deprecated ``firmware_version_from_metadata()`` in favor of ``ouster::sdk::core::SensorInfo::get_version()``.
* [BREAKING] Remove deprecated ``ScanBatcher(size_t, packet_format)`` in favor of ``ScanBatcher(sensor_info)``.
* [BUGFIX] The ``soft_id_check`` for ``SensorScanSource`` was not in use.
* [BREAKING] Remove support for connecting to sensors with firmware versions older than 2.4.0.
* [FUTURE BREAKING] This will be the last release supporting sensor_info, will be deprecated in favor of SensorInfo
* Add ``-g`` or ``--glob`` to ``ouster-cli source`` command which allows globing file names and reading multiple sources of the same type in time order.
* Add ``MultiScanSource`` which can play back multiple ScanSources in time order.
* Add ``--split`` to ``save`` command which can split saved files by size in megabytes.
* Add ``PointViz.set_background_color()`` method to set the background color for PointViz.
* Add full screen support to ``PointViz`` and the ``--fullscreen`` option to the ouster-cli viz.
* Add borderless option to ``PointViz``
* Add ability to hide/show mouse cursor over ``PointViz`` with ``PointViz.cursor_visible(true/false)``.
* Add support for OpenGL ES 3.1 to the viz.
* [BUGFIX] ScanSource reduction no longer crashes reducing LidarScans with non PIXEL_FIELDS.
* Add ``ScanSource.size_hint`` and ``ScanSource.__length_hint__`` to give estimated length of sources.
* Add progress bar to ``ouster-cli source`` to show current progression through the input source. Provide ``ouster-cli source --no-progress`` option to hide.
* Add save command for pointcloud type sources to ``ouster-cli``.
* Add support for new config parameters in FW 3.2 such as ``lidar_frame_azimuth_offset`` and ``bloom_reduction_optimization``
* [BREAKING] ``ScanSource`` iterators now return ``LidarScanSet`` instead of ``std::vector<std::shared_ptr<LidarScan>>``.
* [BUGFIX] Allow integer voxel size in the SLAM/Localization configuration.
* Make SLAM and Localization available using the SDK C++ API.
* PoseOptimizer API Refactor
    - [BREAKING] Removed methods: ``add_absolute_pose_constraint()``, ``add_pose_to_pose_constraint()``, ``add_point_to_point_constraint()``
    - [BREAKING] Per-axis weights: all constraints now support (x, y, z) arrays for rotation and translation weights
    - Add PoseOptimizer new constructor which takes in and loads a constraint JSON file
    - Add Direct constraint construction: Use ``AbsolutePoseConstraint()``, ``PoseToPoseConstraint()``, ``PointToPointConstraint()``
    - Unified constraint-based interface: single ``add_constraint()`` for all constraint types
    - Add ``save_config()`` method to save into a JSON file
    - Add ``remove_constraint()`` method to remove a constraint by the constraint ID
    - Add ``get_constraints()`` and ``set_constraints()`` methods to get and set constraints in the PoseOptimizer
* Support exporting RGB point clouds in PLY and PCD formats.
* Support exporting point cloud normal values in PLY and PCD formats.
* Add ``normals`` command which compute and save per pixel normal values to LidarScan.
* Add ``normals()`` functions for computing surface normals from point cloud geometry shape and range values.
* Support viewing ``LidarScan`` 3 axis dual return Normals fields in viz
* Enable multi-sensor localization in ouster_mapping.
* Add per scan trajectory visualization to the LidarScanViz.
* Add ChanField::WINDOW available in single return, dual return and low bandwidth dual return profiles with firmware 3.2
* Upgrade to KissICP 1.2.3
* Default Sensor sources to not configuring ports with reuse_addr or reuse_port to prevent socket stealing.
* Add ``reuse_ports`` (``--reuse-ports`` in the CLI) to Sensor sources to enable multiple programs to bind to the same sockets simultaneously for some multicast or broadcast applications.
* [BREAKING] Move ``ChanField`` and ``ChanFieldType`` to ``ouster/chanfield.h``.
* [BREAKING] Move ``XYZLutT``, ``XYZLut``, and ``make_xyzlut`` to ``ouster/xyzlut.h``.
* [BREAKING] Make C++ namespaces consistent with Python by nesting under ``ouster::sdk``
    - Moved ouster_client project into ``ouster::sdk::core``
    - Moved ouster_pcap project into ``ouster::sdk::pcap``
    - Moved ouster_mapping project into ``ouster::sdk::mapping``
    - Moved ouster_sensor project into ``ouster::sdk::sensor``
    - Moved ouster_osf project into ``ouster::sdk::osf``
    - Move ``open_source`` and ``open_packet_source`` into ``ouster::sdk`` directly.
* [FUTURE BREAKING] Deprecated DataFormat and UDPProfileLidar overloads of ``get_field_types``
* [BREAKING] Removed ``ouster.sdk.core.default_scan_fields``. Use ``ouster.sdk.util.resolve_field_types`` instead.
* Changed all core sdk enums to enum classes and removed the prefixes from enum values. The C++ enums naming is now consistent with the Python enums naming.
  - e.g. ``OPERATING_NORMAL`` is now ``OperatingMode::UNSPECIFIED``
  - The older enum identiers are still available for use but are deprecated.
* Utilize imu data when compensating for lidar data distortion during motion in SLAM and Localization.
* Add Zone Monitor:
    - Add support for reading and writing Zone Monitor configurations for sensors running firmware 3.2 and above.
    - Add zone state data to LidarScans when Zone Monitor is enabled on the sensor.
    - Add Zone Monitor visualization to the ouster-cli viz command.
    - Add support for emulating Zone Monitor sensor output to the CLI and Python SDK.


Important announcements
-----------------------
* As of 0.16.0, the SDK is no longer compatible with firmware versions older than 2.4.0.

[0.15.0]
========
* The performance of the ``destagger`` method has been improved by two folds.
* [BUGFIX] Fixed crash in SLAM automatic voxel size detection if the first scans have no valid range reading.
* Add pose optimize option to the ouster-cli command to refine the SLAM output.
* [BREAKING] Deprecate ``PointViz::push_frame_buffer_handler()`` and ``PointViz::pop_frame_buffer_handler()``.
  in C++ and Pyton in favor the new screenshot facilities ``get_screenshot(), save_screenshot(), and toggle_screen_recording()``.
* Switch to using ``cartesianT`` as the default ``cartesian`` method implementation for improved performance.
* Add roll camera movement to PointViz using 'Q' and 'E' keys.
* Add finer camera movement to PointViz when using SHIFT modifier for keys 'W', 'A','S','D','Q','E' '-' and '='.
* Repurpose the E key in the viz for roll camera motion. The images size can now be adjusted using 'I/SHIFT + I'.
* [BREAKING] Rename ``window_coordinates_to_image_pixel`` to ``viewport_coordinates_to_image_pixel`` in C++ and Python.

ouster_client/Python SDK
------------------------
* Support opening CDR encoded MCAP files with BagPacketSource and BagScanSource and the CLI.
* [BUGFIX] Fix incorrect range check for LidarScan::complete when start > end.
* Add data_format::packets_per_frame() and data_format::valid_columns_per_frame().
* [BUGFIX] Fix error when decoding OSFs with columns_per_packet != 16.
* Add a new ``save_trajectory`` command to dump the slam output to csv file (TUM format).
* Add a new method ``rotation_matrix_to_quaternion`` to convert a rotation matrix to a quaternion.
* [BREAKING] Rename ouster.sdk.client to ouster.sdk.core.
* [BREAKING] Move get_config, set_config, Sensor, SensorHttp, SensorPacketSource, SensorClient, ClientError,
  ClientOverflow, ClientTimeout from ouster.sdk.client to ouster.sdk.sensor.
* Propogate ``initial_pose`` to SlamEngine and Localization so the reported poses starts from the initial pose.
* Add a new ``filter`` operation to the SDK API and CLI, which filters points based on their projected coordinates.
* Ported ``ScanSource`` and ``open_source`` to C++.
* [BREAKING] Remove ``MultiScanSource`` migrating functionality to ``ScanSource`` which yields List[Optional[LidarScan]].
* [BREAKING] Remove ``PacketMultiSource`` migrating functionality to ``PacketSource`` which yields Tuple[int, Packet].
* [BREAKING] Remove ``SensorScanSource.get_sensor_info()`` in favor of ``SensorScanSource.sensor_info``.
* [BREAKING] Deprecate ``ScanSource.metadata`` and ``PacketSource.metadata`` in favor of ``*.sensor_info``.
* [BREAKING] ``open_source`` or Scan/Packet source constructors throw if unsupported parameters are provided.
* [BREAKING] Remove deprecated ``pcap.Pcap`` ``sensor.Sensor`` and ``osf.Osf`` classes.
* [BREAKING] Deprecated ``ScanSource.sensors_count``. Use length of ``ScanSource.sensor_info`` instead.
* [BREAKING] Remove ``ScanSource.fields`` ``ScanSource.field_types`` ``ScanSource.is_seekable``
* [BREAKING] Remove ``complete`` and ``cycle`` options from ``open_source``. Please check for completeness with ``LidarScan.complete()`` or cycle manually instead.
* [BREAKING] Separate collation/single sensor streams in ``open_source`` into separate ``sensor_idx=int`` and ``collate=bool``. By default sensor_idx=-1 (all sensors) and collate=True (collate).
* Added ``open_packet_source`` function to open packet sources similar to ``open_source``.
* [BREAKING] Deprecate ``ouster.sdk.util.resolve_extrinsics`` for explicitly passing extrinsic file names to sources.
* [BREAKING] Deprecate ``ouster.sdk.sensor.util.build_sensor_config`` for the same functionality already included in SensorScanSource/SensorPacketSource.
* Pin ``kiss-icp`` to ``1.2.2``.
* Remove ``point-cloud-utils`` as a dependency.
* Add ability to set abitrary parameters via ``extra_options`` in the ``SensorConfig``.
* Add support for arbitrary number of addresses per sensor in ``SensorClient``.
* Add generic ``destagger`` to C++ API.
* [BREAKING] Remove ``ouster.sdk.core.FieldDType`` used in type annotations and replaced with ``type``.
* [BREAKING] Remove ``ouster.sdk.core.FieldTypes`` used in type annotations and replaced with ``List[ouster.sdk.coreFieldTypes]``.
* Add a new method `lf_show` to view a set of LidarFrames to the viz API.
* Add python 3.13 support
* [BREAKING] Deprecate ``auto_start_flag`` in favor of ``operating_mode``.
* [BREAKING] Deprecate ``parse_config`` in favor of ``parse_and_validate_config``.
* [BREAKING] Deprecate ``ScanBatcher(size_t w, const sensor::packet_format& pf)`` in favor of ``ScanBatcher::ScanBatcher(const sensor_info&)``.
* [BETA] Add ``ouster.sdk.core.read_pointcloud()`` method to load X Y and Z from PLY and PCD files.
* [BETA] Add ``ouster.sdk.core.voxel_downsample()`` method to downsample pointclouds using a voxel grid.
* Remove dependency on Point Cloud Utils.
* Support opening older ROS 2 bag files that use ouster_msgs/PacketMsg.
* [BUGFIX] Ensure the initial pose matrices are orthonormal before using them in the slam.
* [BREAKING] ``open_source`` ``field_names`` argument now will cause the scan source to load no fields if provided an empty array instead of loading all fields. Provide ``None`` to load all fields.
* Add the ``error_handler`` option to ``open_source`` and ``ScanSourceOptions``, which allows the user to provide a callback to handle error messages for some scan sources.

ouster_cli
----------
* Add additional information on incomplete scans to ``ouster-cli source ... stats``.
* Fix looping behavior of the viz in the CLI when a clip is present.
* The plumb command is now chainable and works with multiple sensors simultaneously.
* Add ``--maximize`` flag to the viz command to maximize the visualizer when launched.
* Modify code to resolve metadata from bag path if ``--meta`` is not provided for rosbag files.
* Add ``--sensor-idx`` argument to source command to allow selecting a single sensor stream.
* Add the ability save LidarScan frames as a series of png images when ``.png`` is specified as the output file for ``save`` command.
* Add the ``--allow-major-version-mismatch`` option to the ``source`` command to allow best-effort loading of OSF files that are not supported by the current SDK version.
* Add GPS auto-constraints support to ``ouster-cli source <osf> pose_optimize`` via ``--auto-constraints``, ``--gps-constraints-every-m``, and ``--gps-constraints-weights``.

ouster_osf
----------
* Improve osf threading model.
* Add version to ``info`` command output and ``Reader`` for OSF files.
* Add an optional error handler callback as an argument the ``Reader`` class constructor.
* Support an optional error handler callback as an option for ``OsfScanSource``.
* ``OsfScanSource`` and ``Reader`` now emit an error or warning if the major or minor version is not supported by the current SDK version.
* [BREAKING] return an ``ouster::util::version`` from ``OsfFile::version()`` instead of an integer.


ouster_viz
----------
* [BUGFIX] Fix AOI label position on macOS.
* Update AOI to allow right click and release without mouse dragging to select a single point.
* [BREAKING] Modify ``WindowCtx::normalized_coordinates`` to operate in viewport coordinates rather than window coordinates.
* [BREAKING] Modify ``WindowCtx::window_coordinates`` to operate in viewport coordinates rather than window coordinates and rename to ``WindowCtx::viewport_coordinates``.
* [BREAKING] Modify ``Image::image_pixel_to_window_coordinates`` to operate in viewport coordinates rather than window coordinates and rename to ``Image::image_pixel_to_viewport_coordinates``.
* Add RING coloring mode to ``LidarScanViz``.
* Add rainbow color palette to ``LidarScanViz``.
* Support (K, 3) and (K, 4) forms for RGB/RGBA key data for Clouds.
* Add screenshot and image recording features to PointViz that allow screenshot size independent of the window size.
* Add a new camera mode ``LidarScanViz.CameraMode.FOLLOW_ROTATION_LOCKED``.
* Discard points with 0 range rather than show them at sensor origin.
* Allow flipping 2D images in ``LidarScanViz`` with CTRL+I.
* Add ``LidarScanViz.select_img_mode`` and ``LidarScanViz.select_cloud_mode`` to select specific fields to display.


Important announcements
-----------------------
* As of 0.13.0, the SDK is no longer compatible with firmware versions older than 2.1.0.
* Official SDK support for firmware versions 2.2 and 2.3 will end at the end of June, 2025.
* Update vcpkg ref of build to 2025.02.14.


[20250117] [0.14.0]
======================

ouster_client/C++ SDK
---------------------

* Jsoncpp fully removed for jsoncons
* [BREAKING] All the HTTP endpoint methods in the ``SensorHttpImp`` class now return a ``std::string`` instead of a ``Json::Value`` object. The result can be parsed with any json parser.
* Add CMake logic for packaging c++ sdk in binary format when ``-DBUILD_SHARED_LIBRARY=ON`` is enabled.

ouster_client/Python SDK
------------------------
* Add a new command ``localize`` to perform localization and tracking within a SLAM-generated map of a given site.
* Add ``LidarScan.sensor_info`` to store the relevant ``SensorInfo`` for each scan
* [BREAKING] Deprecated ``ScanBatcher::operator()(const uint8_t*, uint64_t, LidarScan&)`` for ``ScanBatcher::operator()(const LidarPacket&, LidarScan&)``
* [BREAKING] Disabled ``OUSTER_USE_EIGEN_MAX_ALIGN_BYTES_32`` by default to help avoid ABI mismatches
* [BREAKING] Changed ``SensorClient`` ``get_packet`` API to return packet in the ``ClientEvent`` rather than through reference parameters
* Updated to Kiss ICP 1.1.0 version
* [BUGFIX] Fixed OSF failing to load scans saved in 4096 * 5 mode
* [BUGFIX] Fixed Python ``client.transform`` and ``client.dewarp`` methods returning incorrect results due to ignoring column layout of input data
* Refactored logging to remove spdlog API exposure
* Vendored spdlog in third party dependencies
* [BREAKING] Change ``sensor_info.sn`` type from string to uint64_t
* Support additional array types and formats for Cloud.set_xyz
* Added new ``mask``, ``reduce`` and ``clip`` ScanSource operations to the SDK CLI and API
* The ``clip`` command can now specify which fields to be applied to and accepts unit
* Add relevant methods from ``packet_format`` to ``LidarPacket`` and ``ImuPacket`` classes
* Add ``format`` to each ``Packet`` object with the relevant packet format
* Tolerate off-by-1-byte for bag files recorded using an older version of the ouster-ros driver
* Fix yaw axis to zero in the get_rot_matrix_to_align_to_gravity function
* [BUGFIX] Fix the ``-c`` option in ouster-cli ``config`` command to set config from file

ouster_cli
----------
* Add ``ouster-cli source SENSOR set_static_ip`` command to set sensor static IPs.
* Add ``ouster-cli source SENSOR diagnostics`` command to download sensor diagnostic dumps.
* [BREAKING] Merge the handling of ``--extrinsics-file`` ouster-cli option into ``--extrinsics`` option.
* [BREAKING] ouster-cli ``--extrinsics`` option requires adding double quotes for space separated values.
* ouster-cli ``--extrinsics`` option now accepts ``identity`` as a keyword for overrideng sensor extriniscs with identity.
* ouster-cli ``--extrinsics`` option now accepts the following additional formats besides the ``16`` numbers array format:
  * ``--extrinsics X,Y,Z,R,PY`` for position + euler angles.
  * ``--extrinsics X,Y,Z,QX,QY,QZ,QW`` for position + quaternion.
* Add cursor-driven AOI selection feature to 2d images in ouster-cli ``viz`` command.

ouster_osf
----------
* Introduce ``ouster::osf::AsyncWriter`` to offload saving ``LidarScan`` as OSF to a background thread, improving CLI performance when saving OSF files.
* Add the ``ouster::osf::Encoder`` type, which allows parameterizing the OSF compression level.
* Change the default OSF PNG encoder compression level to 1 from 4.
* [BREAKING] ``ouster.sdk.osf`` no longer exports lower-level OSF API classes (such as ``osf.Reader``.)
* ``ouster::osf::Writer::save`` now throws if the resolution of a ``LidarScan`` being saved doesn't match what is specified in sensor info/metadata.
* [BUGFIX] Fix incorrect ``OsfScanSource`` data when reading from an OSF file containing empty or missing streams.

ouster_viz
----------
* ``SimpleViz`` now drops frames when necessary to keep up with a live data source (i.e. sensor.)
* Add a map origin axis and label.
* Invoke frame buffer resize handlers added to ``PointViz`` when GLFW's window resize event fires.
* [BREAKING] Change ``PointViz::window_coordinates_to_image_pixel`` so that it always returns a pixel location (even outside the image), which can be useful in some situations.
* [BUGFIX] On screen display frame number starts at zero instead of one.
* [BUGFIX] ``LidarScanViz`` now only creates view modes for PIXEL fields.
* [BUGFIX] Use the last valid column pose as a ``LidarScan``'s origin, instead of the first.
* [BUGFIX] Limit number of keyboard shortcuts to toggle sensors from CTRL+1 to CTRL+9.
* [BUGFIX] Fix a key shortcut help rendering issue and improve consistency of key shortcut help.


[20241017] [0.13.1]
======================

* Add support for directly using IPv6 addresses for sensors in the CLI and in sensor clients.
* Typing '?' now displays the visualizer keyboard shortcuts in the visualizer window.
* Removed the ``async_client_example.cpp`` example.
* Un-deprecated ``ScanBatcher::ScanBatcher(size_t, const packet_format&)`` to remove a warning. (But please use ``ScanBatcher::ScanBatcher(const sensor_info&)`` instead.)

* [BREAKING] Removed the ``input_row_major`` parameter from the ``dewarp`` function. (``dewarp`` now infers the array type.)
* [BREAKING] Renamed ``DEFAULT_HTTP_REQUEST_TIMEOUT_SECONDS`` to ``LONG_HTTP_REQUEST_TIMEOUT_SECONDS``.
* [BREAKING] Changed the default value of ``LidarScanVizAccumulatorsConfig.accum_min_dist_num`` from ``1`` to ``0``.
* [BUGFIX] Fixed a visualizer glitch causing drawables not to render if added after a call to ``PointViz::update()`` but before ``PointViz::run()`` or ``PointViz::run_once()``.
* [BUGFIX] Fixed a visualizer crash when using ``HIGHLIGHT_SECOND`` mode with single-return datasets.
* [BUGFIX] Fixed an issue with the 2d images not updating when cycled during pause.
* [BUGFIX] Fixed a bug that the first scan pose it not identity when using slice slam command on a slam output osf file
* [BUGFIX] Re-introduce the RAW field option

Known Issues
------------

* Using an unbounded slice (e.g. with ``slice 100:`` during visualization can cause the source to loop back to the beginning (outside of the slice) when the source is a pcap file or an OSF saved with an earlier version of the SDK.
* A race condition in ``PointViz`` event handers occasionally causes a crash or unexpected results.


[20240702] [0.13.0]
======================

ouster_osf
------------------------
* Add full index of both receive and sensor timestamps to metadata
* Speed up opening of OSF files with index

* OSF now saves alert flags, thermal countdown and status, shot limiting countdown and status from ``LidarScan``.
* [BUGFIX] Fix OSF being unable to load LidarScans containing only custom fields
* [BUGFIX] Fix OSF not flushed when the user pressed CTRL-C more than once
* [BUGFIX] Fix improper timestamps when saving OSF on MacOS(m-series) and Windows
* [BUGFIX] Fix an issue with destaggering images after modifying ``SensorInfo`` in an ``OsfScanSource``.
* [BUGFIX] Fix an issue loading extrinsics from OSF metadata into a ``SensorInfo`` in ``OsfScanSource``.
* [BREAKING] Remove ``ChunksLayout`` and ``ChunkRef`` from Python API.

ouster_client/Python SDK
------------------------

* Add support for reading and writing ROS1 and ROS2 bag files.
* Add new sensor client interface ``ouster::sensor::SensorClient`` which supports multiple sensors as well as multiple sensors and IMU data on the same port
* Add higher level sensor client interface ```ouster::sensor::SensorScanSource`` which produces ``LidarScan`` s from multiple sensors
* Add ``ouster.sdk.client.SensorPacketSource`` which receives packets from multiple sensors
* Add support for multiple sensors to ``ouster.sdk.sensor.SensorScanSource``
* Greatly reduced redundant HTTP API calls to the sensor during initialization
* Deserialize FLAGS fields in each profile by default
* Add support for IPv6 multicast
* Add ``field_names`` argument to each scan source and to ``open_source`` to specify which fields to decode
* Add metadata validation functionality
* Add vendored json library
* Improved multi sensor pcap reading
* Improve ``ScanBatcher`` to release ``LidarScan`` as soon as they are completed
* ``ScanBatcher`` now adds alert flags, thermal countdown, and shot limiting countdown to ``LidarScan``.
* Use index to speed up ``ouster-cli source .osf info``
* Use index to speed up slicing of indexed OSF sources when sliced immediately after the ``source`` command
* Add ``LidarScan.get_first_valid_column_timestamp()``
* Add ``crc`` and ``calculate_crc`` methods to ``ouster::sensor::packet_format`` for obtaining or calculating (respectively) the CRC64 of a packet.
* ``scan_to_packets`` now creates packets with alert flags, thermal countdown and status, shot limiting countdown and status, and CRC64.
* Add ``ouster::pose_util::dewarp`` C++ function to de-warp a ``LidarScan`` (similar to ``ouster.sdk.pose_util`` in the Python API.)
* Add a constructor ``LidarScan(const ouster::sensor::sensor_info&)``.
* Always use ``nonstd::optional`` instead of drop-in ``std::optional`` from https://github.com/martinmoene/optional-lite.git to reduce issues associated with mixing C++14 and C++17.
* Add ``w()`` and ``h()`` methods to ``sensor_info`` in C++ and ``w`` and ``h`` properties to ``SensorInfo`` in Python.
* [BUGFIX] fix automatic UDP dest for FW 2.3 sensors.


* [BREAKING] Remove ``ouster::make_xyz_lut(const ouster::sensor::sensor_info&)``. (Use ``make_xyz_lut(const sensor::sensor_info& sensor, bool use_extrinsics)`` instead.)
* [BREAKING] changed REFLECTIVITY channel field size to 8 bits. (Important - this makes the SDK incompatible with FW 2.0.)
* [BREAKING] Removed ``UDPPacketSource`` and ``BufferedUDPSource``.
* [BREAKING] Removed ``ouster.sdk.util.firmware_version(hostname)`` please use ``ouster.sdk.client.SensorHttp.create(hostname).firmware_version()`` instead
* [BREAKING] ``open_source`` no longer automatically finds and applies extrinsics from ``sensor_extrinsics.json`` files. Use the ``extrinsics`` argument instead to specify the path to the relevant extrinsics file instead.
* [BREAKING] Deprecated ``osf.Scans(...)`` for ``osf.OsfScanSource(...).single_source(0)```.
* [BREAKING] Deprecated ``client.Sensor(...)`` for ``client.SensorPacketSource(...).single_source(0)```.
* [BREAKING] Deprecated ``pcap.Pcap(...)`` for ``pcap.PcapMultiPacketReader(...).single_source(0)```.
* [BREAKING] Deprecated ``ScanBatcher::ScanBatcher(size_t, const packet_format&)`` for ``ScanBatcher::ScanBatcher(const sensor_info&)``.
* [FUTURE BREAKING] Removing all instances of jsoncpp's ``Json::Value`` from the public C++ API methods in favor of ``std::string``.

ouster_viz
----------

* ``LidarScanViz`` now supports multi-sensor datasets.
* Add Python callback registration methods for mouse button and scroll events from ``PointViz``.
* Add Python and C++ callback registration methods for frame buffer resize events.
* Add ``MouseButton``, ``MouseButtonEvent``, and ``EventModifierKeys`` enums.
* Add methods ``aspect_ratio``, ``normalized_coordinates``, and ``window_coordinates`` to ``viz::WindowCtx``.
* Add method ``window_coordinates_to_image_pixel`` to ``viz::Image``. (See ``viz_events_example.cpp`` for an example.)
* Add ``current_camera()`` method to ``PointViz``.
* [BREAKING] ``SimpleViz`` no longer accepts a ``ScansAccumulator`` instance and now accepts scan/map accumulation parameters as keyword args in its constructor.
* [BREAKING] ``ScansAccumulator`` is split into several different classes: ``ScansAccumulator``, ``MapAccumulator``, ``TracksAccumulator``, and ``LidarScanVizAccumulators``.
* [BREAKING] changed ``PointViz`` mouse button callback to fire for both mouse button press and release events.
* [BREAKING] changed ``PointViz`` mouse button callback signature to use the new enums.
* [BREAKING] removed ``bool update_on_input()`` and ``update_on_input(bool)`` methods from ``PointViz``.
* [BUGFIX] SimpleViz throws a 'generator already executing' exception.

ouster-cli
----------

* Add support for reading and writing ROS1 and ROS2 bag files.
* Add support for working with multi scan sources.
* Add ``--fields`` argument to ``ouster-cli source`` to specify which fields to decode.
* Add metadata validation utility.
* [BUGFIX] Program doesn't terminate immediately when pressing CTRL-C the first time when streaming from a live sensor.
* [BUGFIX] Fix some errors that appeared when running ``ouster-cli util benchmark``
* [BREAKING] ``source`` no longer automatically finds and applies extrinsics from ``sensor_extrinsics.json`` files. Use the ``-E`` argument instead to specify the path to the relevant extrinsics file instead.
* [BREAKING] Moved raw recording functionality for BAG and PCAP to ``ouster-cli source ... record_raw`` command.
* [BREAKING] CLI plugins now need to handle a list of Optional[LidarScan] instead of a single LidarScan to support multi sources.

mapping
-------

* Update KissICP version from 0.4.0 to 1.0.0.
* Add multi-sensor support.

[20240702] [0.12.0]
===================

**Important: ouster-sdk installed from pypi now requires glibc >= 2.28.**

ouster_client/Python SDK
------------------------

* Add support for adding custom fields to ``LidarScan`` s with ``add_field`` and ``del_field``
* Added per-request timeout arguments to ``SensorHttp``
* Added sensor user_data to ``sensor_info/SensorInfo`` and metadata files
* Removed ``updated_metadata_string()`` and ``original_string()`` from ``sensor_info``
* Added ``to_json_string()`` to ``sensor_info`` to convert a ``sensor_info`` to a non-legacy
  metadata JSON string
* Unified Python and C++ ``Packet`` and ``PacketFormat`` classes
* Added ``validate`` function to ``LidarPacket`` and ``ImuPacket`` to check for ID and size mismatches
* [BREAKING] LidarScan's width and height have been switched to size_t from ptrdiff_t in C++
* Refactor metadata parsing
* Add ``get_version`` to ``sensor_info/SensorInfo`` to retrieve parsed version information
* Add ``get_product_info`` to ``sensor_info/SensorInfo`` to retrieve parsed lidar model information
* Raise an exception rather than throw an unrelated error when multiple viable metadata files are found for a given PCAP
* Add ability to slice a scan source, returning a new sliced ScanSource

* [BREAKING] Removed ``hostname`` in Python ``SensorInfo`` and ``name`` from C++ ``sensor_info``
* [BREAKING] Removed ``udp_port_lidar``, ``udp_port_imu`` and ``mode`` from C++ ``sensor_info``
* [BREAKING] Deprecated ``udp_port_lidar``, ``udp_port_imu`` and ``mode`` in Python ``SensorInfo``.
  These fields now point to the equivalent fields inside of ``SensorInfo::config``.
* [BREAKING] Removed ``cols`` and ``frequency`` from ``LidarMode`` in Python
* [BREAKING] Deprecated ```data``` and ``capture_timestamp`` from Python ``Packet``
* [BREAKING] Removed methods from Python ``ImuPacket`` and ``LidarPacket`` classes that simply wrapped ``PacketFormat``
* [BREAKING] Removed ``begin()`` and ``end()`` iterators of ``LidarScan`` in C++
* [BREAKING] Remove deprecated package stubs added in previous 0.11 release.
* [BREAKING] Replaced integer backed ``ChanField`` enumerations with strings.
* [BREAKING] Removed ``CUSTOM0`` through ``CUSTOM9`` ChanField enumerations.
* [BREAKING] Extra fields in sensor metadata are now ignored and discarded if saved from the resulting ``sensor_info/SensorInfo``

* [BUGFIX] Prevent last scan from being emitted twice for PCAP
* [BUGFIX] Fix corrupted packets due to poor handling of fragmented packet drop in PCAPs
* [BUGFIX] Fix possible crash when working with custom UDPProfileLidars

ouster_viz
----------
* Support viewing custom ``LidarScan`` fields in viz
* Support viewing custom ``LidarScan`` 3 channel fields in viz as RGB

* [BUGFIX] Prevent OpenBLAS from using high amounts of CPU spin waiting

ouster_osf
----------

* Support saving custom ``LidarScan`` fields to OSF files

* [BREAKING] OsfWriter now takes in an optional list of fields to save rather than a list of fields and ChanFieldTypes to cast to

ouster-cli
----------

* Added support for slicing using time to ``ouster-cli source ... slice``
* Add sensor ``ouster-cli source ... userdata`` command to set and retrieve userdata on a sensor
* Add chainable ``ouster-cli source ... stats`` command
* Add chainable ``ouster-cli source ... clip`` command to discard points outside a provided range
* Add ``--rate max`` option to ``ouster-cli source ... viz```
* Improve argument naming and descriptions for ``ouster-cli source ... viz`` map and accum options:
  ``--accum-map`` is now called ``--map`` and ``--accum-map-ratio`` is now called ``--map-ratio``.
* New ``--map-size`` argument to set the maximum number of points used when ``--map`` is specified.

* [BUGFIX] Prevent dropped frames from live sensors by consuming scans as fast as they come in rather than sleeping

mapping
-------

* Move mapping into the sdk as ``ouster.sdk.mapping``
* Better handle looping while mapping
* Improve automatic downsample voxel size calculation

other
-----

* Updated VCPKG libraries to 2024.04.26

[20240510] [0.11.1]
===================

Important notes
---------------

* [BREAKING] the ``open_source`` method now returns a ``ScanSource`` by default, not a ``MultiScanSource``.

Python SDK
----------

* Updated the ``open_source`` documentation.
* Fixed an issue that caused the viz to redraw when the mouse cursor is moved.
* [BREAKING] The python slice ``[::]`` operator now returns a ``MultiScanSource`` / ``ScanSource``
  instead of a ``List``. Thus, invoking a ``scan_source[x:x+n]`` yields a fully functional scan source
  that is scoped to the range ``[x, x+n]``.
* [BREAKING] The python slice ``[::]`` operator no longer support negative step

ouster_client
-------------

* Improved the client initialization latency.

mapping
-------

* Fixed several issues with the documentation.


[20240425] [0.11.0]
===================

Important notes
---------------

* Dropped support for python3.7
* Dropped support macOS 10.15
* This will be the last release that supports Ubuntu 18.04.
* Moved all library level modules under ``ouster.sdk``, this includes ``ouster.client``, ``ouster.pcap``
  ``ouster.osf``. So the new access name will be ``ouster.sdk.client``, ``ouster.sdk.pcap`` and so on
* [BREAKING] many of the ``ouster-cli`` commands and arguments have changed (see below.)
* [BREAKING] moved ``configure_sensor`` method to ``ouster.sdk.sensor.util`` module
* [BREAKING] removed the ``pcap_to_osf`` method.


examples
--------

* Added a new ``async_client_example.cpp`` C++ example.


Python SDK
----------

* Add support for python 3.12, including wheels on pypi
* Updated VCPKG libraries to 2023.10.19
* New ``ScanSource`` API:

   * Added new ``MultiScanSource`` that supports streaming and manipulating LidarScan frames from multiple concurrent LidarScan sources

     * For non-live sources the ``MultiScanSource`` has the option to choose LidarScan(s) by index or choose a subset of scans using slicing operation
     * The ``MultiScanSource`` interface has the ability to fallback to ``ScanSource`` using the ``single_source(sensor_idx)``, ``ScanSource`` interface yield a single LidarScan on iteration rather than a List
     * The ``ScanSource`` interface obtained via ``single_source`` method supports same indexing and and slicing operations as the ``MultiScanSource``

  * Added a generic ``open_source`` that accepts sensor urls, or a path to a pcap recording or an osf file
  * Add explicit flag ``index`` to index unindexed osf files, if flag is set to ``True`` the osf file
    will be indexed and the index will be saved to the file on first attempt
  * Display a progress bar during index of pcap file or osf (if unindexed)

* Improved the robustness of the ``resolve_metadata`` method used to
  automatically identify the sensor metadata associated with a PCAP source.
* [bugfix] SimpleViz complains about missing fields
* [bugfix] Gracefully handle failed sensor connection attempts with proper error reporting
* [bugfix] Fix assertion error when using viz stepping on a live sensor
* [bugfix] Scope MultiLidarViz imports to viz commands
* [bugfix] LidarScan yielded with improper header/status
* [bugfix] OSF ScanSource fields property doesn't report the actual fields
* Removed ``ouster.sdkx``, the ``open_source`` command is now part of ``ouster.sdk`` module
* The ``FLAGS`` field is always added to the list fields of any source type by default. In case of a
  dual return lidar profile then a second ``FLAGS2`` will also be added.


mapping
-------

* Updated SLAM API and examples.
* Added real time frame dropping capability to SLAM API.
* The ``ouster-mapping`` package now uses ``point-cloud-utils`` instead of ``open3d``.
* improved per-column pose accuracy, which is now based on the actual column timestamps


ouster-cli
----------

* Many commands can now be chained together, e.g. ``ouster-cli source <src> slam viz``.
* New ``save`` command can output the result in a variety of formats.
* Added ``--ts`` option for specifying the timestamps to use when saving an OSF
  file. Host packet receive time is the default, but not all scan sources have
  this info. Lidar packet timestamps can be used as an alternative.
* Changed the output format of ``ouster-cli discover`` to include more information.
* Added JSON format output option to ``ouster-cli discover``.
* Added a flag to output sensor user data to ``ouster-cli discover``.
* Update the minimum required version of ``zeroconf``.
* Removed ``python-magic`` package from required dependencies.
* Made the output of ``ouster-cli source <osf> info`` much more
  user-friendly. (``ouster-cli source <osf> dump`` gives old output.)
* [breaking] changed the argument format of the ``slice`` command.
* [breaking] removed the ``--legacy`` and ``--non-legacy`` flags.
* [breaking] removed the ``ouster-cli mapping``, ``ouster-cli osf``,
  ``ouster-cli pcap``, and ``ouster-cli sensor`` commands.
* [bugfix] return a nonzero exit code on error.
* [bugfix] fix an error that occurred when setting the IMU port using the
  ``-i`` option.


ouster_client
-------------

* Added a new buffered UDP source implementation ``BufferedUDPSource``.
* The method ``version_of_string`` is marked as deprecated, use ``version_from_string``
  instead.
* Added a new method ``firmware_version_from_metadata`` which works across firmwares.
* Added support for return order configuration parameter.
* Added support for gyro and accelerometer FSR configuration parameters.
* [bugfix] ``mtp_init_client`` throws a bad optional access.
* [bugfix] properly handle 32-bit frame IDs from the
  ``FUSA_RNG15_RFL8_NIR8_DUAL`` sensor UDP profile.


ouster_osf
----------

* [breaking] Greatly simplified OSF writer API with examples.
* [breaking] removed the ``to_native`` and ``from_native`` methods.
* Updated Doxygen API documentation for OSF C++ API.
* Removed support for the deprecated "standard" OSF file format. (The streaming
  OSF format is still supported.)
* Added ``osf_file_modify_metadata`` that allows updating the sensor info
  associated with each lidar stream in an OSF file.
* Warn the user if reading an empty or improperly indexed file.


ouster_viz
----------
* Added scaled palettes for calibrated reflectivity.
* Distance rings can now be hidden by setting their thickness to zero.
* [bugfix] Fix some rendering issues with the distance rings.
* [bugfix] Fix potential flickering in Viz


Known issues
------------

* ouster-cli discover may not provide info for sensors using IPv6 link-local
  networks on Python 3.8 or with older versions of zeroconf.
* ouster-cli when combining ``slice`` command with ``viz`` the program will
  exit once iterate over the selected range of scans even when
  the ``--on-eof`` option is set to ``loop``.

  - workaround: to have ``viz`` loop over the selected range, first perform a
    ``slice`` with ``save``, then playback the generated file.


[20231031] [0.10.0]
===================

Important notes
---------------

* This will be the last release that supports Python 3.7.
* This will be the last release that supports macOS 10.15.

ouster_viz
----------

* Added point cloud accumulation support
* Added an ``PointViz::fps()`` method to return the operating frame rate as a ``double``

ouster_client
-------------

* [BREAKING] Updates to ``sensor_info`` include:
    * new fields added: ``build_date``, ``image_rev``, ``prod_pn``, ``status``, ``cal`` (representing the value stored in the ``calibration_status`` metadata JSON key), ``config`` (representing the value of the ``sensor_config`` metadata JSON key)
    * the original JSON string is accessible via the ``original_string()`` method
    * The ``updated_metadata_string()`` now returns a JSON string reflecting any modifications to ``sensor_info``
    * ``to_string`` is now marked as deprecated
* [BREAKING] The RANGE field defined in `parsing.cpp`, for the low data rate profile, is now 32 bits wide (originally 16 bits.)
    * Please note this fixes a SDK bug. The underlying UDP format is unchanged.
* [BREAKING] The NEAR_IR field defined in `parsing.cpp`, for the low data rate profile, is now 16 bits wide (originally 8 bits.)
    * Plase note this fixes a SDK bug. The underlying UDP format is unchanged.
* [BREAKING] changed frame_id return size to 32 bits from 16 bits
* An array of per-packet timestamps (called ``packet_timestamp``) is added to ``LidarScan``
* The client now retries failed requests to an Ouster sensor's HTTP API
* Increased the default timeout for HTTP requests to 40s
* Added FuSA UDP profile to support Ouster FW 3.1+
* Improved ``ScanBatcher`` performance by roughly 3x (depending on hardware)
* Receive buffer size increased from 256KB to 1MB
* [bugfix] Fixed an issue that caused incorrect Cartesian point computation in the ``viz.Cloud`` Python class
* [bugfix] Fixed an issue that resulted in some ``packet_format`` methods returning an uninitialized value
* [bugfix] Fixed a libpcap-related linking issue
* [bugfix] Fixed an eigen 3.3-related linking issue
* [bugfix] Fixed a zero beam angle calculation issue
* [bugfix] Fixed dropped columns issue with 4096x5 and 2048x10

ouster-cli
----------

* Added ``source <FILE> slam`` and ``source <FILE> slam viz`` commands
* All metadata CLI options are changed to ``-m/--metadata``
* Added discovery for FW 3.1+ sensors
* Set signal multiplier by default in sensor/SOURCE sensor config
* use ``PYBIND11_MODULE`` instead of deprecated module constructor
* remove deprecated == in pybind for ``.is()``
* [bugfix] Fix report of fragmentation for ouster-cli pcap/SOURCE pcap info
* [bugfix] Fixed issue regarding windows mDNS in discovery
* [bugfix] Fixed cli pcap recording timestamp issue
* [BREAKING] CSV output ordering switched

ouster.sdk
----------

* ``ouster-mapping`` is now a required dependency
* [BREAKING] change the ``ouster.sdk.viz`` location to the ``ouster.viz``
  package, please update the references if you used ``ouster.sdk.viz`` module
* [bugfix] Fixed Windows pcap support for files larger than 2GB
* [bugfix] Fixed the order of ``LidarScan``'s ``w`` and ``h`` keyword arguments
* [bugfix] Fixed an issue with ``LidarPacket`` when using data recorded with older versions of Ouster Studio

Known issues
------------

* The dependency specifier for ``scipy`` is invalid per PEP-440
* ``get_config`` always returns true
* Repeated CTRL-C can cause a segmentation fault while visualizing a point cloud

20230710
========

* Update vcpkg ref of build to 2023-02-24

ouster_osf
----------

* Add Ouster OSF C++/Python library to save stream of LidarScans with metadata

ouster_client
-------------

* Add ``LidarScan.pose`` with poses per column
* Add ``_client.IndexedPcapReader`` and ``_client.PcapIndex`` to enable random pcap file access by frame number
* [BREAKING] remove ``ouster::Imu`` object
* [BREAKING] change the return type of ``ouster::packet_format::frame_id`` from ``uint16_t`` to ``uint32_t``
* [BREAKING] remove methods ``px_range``, ``px_reflectivity``, ``px_signal``, and ``px_ambient`` from ``ouster::packet_format``
* Add ``get_field_types`` function for ``LidarScan``, from ``sensor_info``
* bugfix: return metadata regardless of ``sensor_info`` status field
* Make timeout on curl more configurable
* [BREAKING] remove encoder_ticks_per_rev (deprecated)

ouster_viz
----------

* [BREAKING] Changed Python binding for ``Cloud.set_column_poses()`` to accept ``[Wx4x4]`` array
  of poses, column-storage order
* bugfix: fix label re-use
* Add ``LidarScan.pose`` handling to ``viz.LidarScanViz``, and new ``T`` keyboard
  binding to toggle column poses usage

ouster_pcap
-----------
* bugfix: Use unordered map to store stream_keys to avoid comparison operators on map

Python SDK
----------
* Add Python 3.11 wheels
* Retire simple-viz for ouster-cli utility
* Add default ? key binding to LidarScanViz and consolidate bindings into stored definition
* Remove pcap-to-csv for ouster-cli utility
* Add validator class for LidarPacket

ouster-cli
----------
This release also marks the introduction of the ouster-cli utility which includes, among many features:
* Visualization from a connected sensor with automatic configuration
* Recording from a connected sensor
* Simultaneous record and viz from a connected sensor
* Obtaining metadata from a connected sensor
* Visualization from a specified PCAP
* Slice, info, and conversion for a specificed PCAP
* Utilities for benchmarking system, printing system-info
* Discovery which indicates all connected sensors on network
* Automatic logging to .ouster-cli
* Mapping utilities


[20230403]
==========

* Default metadata output across all functionality has been switched to the non-legacy format

ouster_client
-------------
* Added a new method ``mtp_init_client`` to init the client with multicast support (experimental).
* the class ``SensorHttp``  which provides easy access to REST APIs of the sensor has been made public
  under the ``ouster::sensor::util`` namespace.
* breaking change: get_metadata defaults to outputting non-legacy metadata
* add debug five_word profile which will be removed later
* breaking change: remove deprecations on LidarScan

ouster_viz
----------
* update viz camera with other objects in draw

ouster_pcap
-----------
* add seek method to PcapReader
* add port guessing logic

python
------
* introduce utility to convert nonlegacy metadata to legacy
* use resolve_metadata to find unspecified metadata for simple-viz
* remove port guessing logic in favor of using new C++ ouster_pcap port guessing functionality
* add soft-id-check to skip the init_id/sn check for lidar_packets with metadata

Numerous changes to SimpleViz and LidarScanViz including:
* expose visible in viz to Python
* introduce ImageMode and CloudMode
* bugfix: remove spurious sqrt application to autoleveled images


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
