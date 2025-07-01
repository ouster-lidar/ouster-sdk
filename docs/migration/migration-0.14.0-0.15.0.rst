================================
Migration from 0.14.0 to 0.15.0
================================

The OusterSDK 0.15.0 release introduces several breaking changes. This guide summarizes how to migrate your code from 0.14.0 to 0.15.0.

Rename of ``ouster.sdk.client`` to ``ouster.sdk.core``
++++++++++++++++++++++++++++++++++++++++++++++++++++++

The module ``ouster.sdk.client`` has been renamed to ``ouster.sdk.core``.

Update your imports accordingly. For example:

.. code:: python

   # Old code:
   from ouster.sdk.client import SensorInfo

   # New code:
   from ouster.sdk.core import SensorInfo

Relocation of select ``sdk.client`` classes and functions to ``sdk.sensor``
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Additionally, several classes and methods have been moved to ``ouster.sdk.sensor``.

- ``get_config``, ``set_config`` have been moved to ``ouster.sdk.sensor``.
- ``Sensor``, ``SensorHttp``, ``SensorPacketSource``, ``SensorClient``, ``ClientError``, ``ClientOverflow``,  and ``ClientTimeout`` have been moved to ``ouster.sdk.sensor``.


Update your usage accordingly. For example:

.. code:: python

   # Old code:
   from ouster.sdk.client import Sensor

   # New code:
   from ouster.sdk.sensor import Sensor

Removal of ``MultiScanSource``
++++++++++++++++++++++++++++++

``MultiScanSource`` has been removed. Its functionality is now handled by ``ScanSource``,
which returns a ``List[Optional[LidarScan]]``, if ``sensor_idx < 0`` and ``collated=True`` which is the default.
This is the case for both single and multiple sensor when the parameters are as mentioned above.

If ``sensor_idx >= 0`` or ``collated`` is set to ``False``, the returned list is of length 1
and contains ``LidarScan`` for a single sensor

If you previously used below to get single sensor scans:

.. code:: python

    source = open_source(pcap_path, meta=[metadata_path])


You can now use:

.. code:: python

    source = open_source(pcap_path, meta=[metadata_path], sensor_idx=0, collate=False)

Removal of ``single_source(sensor_idx)``
++++++++++++++++++++++++++++++++++++++++

Prior to 0.15.0, ``single_source(sensor_idx)`` was used to limit the output to ``Iterable`` of scans for a single sensor with the specified sensor index.

As of 0.15.0, ``SensorScanSource`` always returns a list of lidar scans regardless of single and  multiple sensors.
The list will typically contain one ``LidarScan`` per sensor.

If your setup involves only a single sensor, can now use below to remove the call to ``single_source(sensor_idx)`` entirely:

.. code:: python

    for scans in source:
        scan = scans[0]

If you prefer to explicitly select a single sensor by index, use ``single(sensor_idx)`` instead:

.. code:: python

    # Old code:
    for scan in source.single_source(sensor_idx):
        ...

    # New code:
    for scan in source.single(sensor_idx):
        ...

Refer to the :ref:`python quickstart <iterating-over-scans>` for more information.


Removal of ``PacketMultiSource``
++++++++++++++++++++++++++++++++

``PacketMultiSource`` has been removed. It's functionality is now handled by ``PacketSource``,
which returns a ``Tuple[int, Packet]``. This represents both single and multi sensor data streams.

Previously:

.. code:: python

    class PacketMultiSource(Protocol):
        """Represents a multi-sensor data stream."""

    def __iter__(self) -> Iterator[Tuple[int, Packet]]:
        """A PacketSource supports ``Iterable[Tuple[int, Packet]]``."""


Definition now:

.. code:: python

    class PacketSource:
        def __iter__(self) -> Iterator[Tuple[int, Union[LidarPacket, ImuPacket]]]:
            ...


Removal of ``SensorScanSource.get_sensor_info()``
+++++++++++++++++++++++++++++++++++++++++++++++++

The method ``SensorScanSource.get_sensor_info()`` has been removed.

**Replacement:**  Access metadata using the ``SensorScanSource.sensor_info`` attribute instead.


Updates to Error Handling on unsupported parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++

As of version 0.15.0, ``open_source`` and the constructors for ``ScanSource`` and ``PacketSource`` will now throw an error if unsupported parameters are provided.
This stricter validation ensures that only supported arguments are passed to these functions.

For example:

.. code:: python

   source = open_source(path=pcap_path, meta=[metadata_path])  # No error

   source = open_source(path=pcap_path, meta=[metadata_path], cycle=True)  # Raises an exception since cycle is now unsupported


Removal of previously deprecated classes
++++++++++++++++++++++++++++++++++++++++

The following classes were deprecated in 0.14.0 and are now removed in 0.15.0:

- ``pcap.Pcap``
- ``sensor.Sensor``
- ``osf.Osf``

Use ``PcapPacketSource`` instead of ``pcap.Pcap``. If you were using:

.. code:: python

    # Old code:
    packets = pcap.Pcap(file,
                        info)
    # New code:
    packets = pcap.PcapPacketSource(file,
                            sensor_info=[info])


Use ``SensorPacketSource`` instead of ``Sensor``. If you were using:

.. code:: python

    # Old code:
    src = sensor.Sensor("test-sensor",
                        7502,
                        7503,
                        metadata=info,
                        _flush_before_read=False)
    # New code:
    src = sensor.SensorPacketSource("test-sensor",
                        sensor_info=[info])


Removal of select attributes from ``ScanSource``
++++++++++++++++++++++++++++++++++++++++++++++++

- ``is_seekable`` has been removed from ``ScanSource``. You can achieve similar results using ``!is_live``.

- Additionally, ``fields`` and ``field_types`` were removed from ``ScanSource``. This is now handled on ``LidarScan`` since the values can vary per scan.

.. code:: python

   # New code:
   for scan, in src:
    print(scan.fields, scan.field_types)


Removal of select options from ``open_source``
++++++++++++++++++++++++++++++++++++++++++++++

- The option ``complete`` has been removed from ``open_source``. Please use ``LidarScan.complete()`` to check scan for completeness.

.. code:: python

    for s, in scans:
        if not s.complete(col_window):
            logger.warning(f"Received incomplete frame")

- The option ``cycle`` options from ``open_source``. Please use the ``on_eof='loop`` in SimpleViz constructor instead or cycle manually instead.

.. code:: python

    # New code:
    SimpleViz(scans.sensor_info,
            pause_at=0, on_eof='loop', rate=1.0).run(scans)


Removal of type annotations in ``ouster.sdk.core``
++++++++++++++++++++++++++++++++++++++++++++++++++

``ouster.sdk.core.FieldDType`` used in type annotations has been replaced with ``type``.
In addition, ``ouster.sdk.core.FieldTypes`` has been removed and replaced with ``List[ouster.sdk.core.FieldTypes]``.

This is to align SDK with Python's native type system.


Changes to Window and Image Coordinates
+++++++++++++++++++++++++++++++++++++++

The following methods now work in viewport coordinates instead of window coordinates:

- ``WindowCtx::normalized_coordinates`` now operates in viewport coordinates.
- ``WindowCtx::window_coordinates`` has been renamed to ``WindowCtx::viewport_coordinates``.
- ``Image::image_pixel_to_window_coordinates`` has been renamed to ``Image::image_pixel_to_viewport_coordinates``.

Update your code to reflect these naming and functionality changes. For example:

.. code:: python

   # Old code:
   coords = window_ctx.window_coordinates(pixel)

   # New code:
   coords = window_ctx.viewport_coordinates(pixel)


Updates to metadata retrieval
++++++++++++++++++++++++++++++

From 0.15.0, ``ScanSource.metadata`` and ``PacketSource.metadata`` have been deprecated and will be removed in the future.

Please use ``ScanSource.sensor_info`` and ``PacketSource.sensor_info`` for the same information.


Deprecated ``sensors_count``
+++++++++++++++++++++++++++++

In version 0.15.0, the attribute ``ScanSource.sensors_count`` has been deprecated and will be removed in a future release.

Use the length of ``ScanSource.sensor_info`` instead. For example:

.. code:: python

   # Old code:
   num_sensors = scan_source.sensors_count

   # New code:
   num_sensors = len(scan_source.sensor_info)


Deprecated ``multi.collate_scans``
++++++++++++++++++++++++++++++++++

The function ``ouster.sdk.core.multi.collate_scans`` has been deprecated.

**Replacement:**
Use ``ouster.sdk.core.collate`` instead.


Deprecated ``resolve_extrinsics``
+++++++++++++++++++++++++++++++++

The function ``resolve_extrinsics`` will be removed in a future release. You should now explicitly pass the extrinsics file when constructing a scan source.
To do this, use an extrinsics file similar to below, with 12225000xxxx replaced with your sensor serial number:

    .. code:: console

        {
            "transforms": [
            {
                "destination_frame": "world",
                "p_x": -110.27235412597656,
                "p_y": 0.29306289553642273,
                "p_z": 6.037787437438965,
                "q_w": 0.5709517002105713,
                "q_x": -0.18786071240901947,
                "q_y": 0.16413573920726776,
                "q_z": 0.7821647524833679,
                "source_frame": "12225000xxxx"
            }
            ]
        }

  Then, use open_source with the extrinsics file:

    .. code:: python

        source = open_source(source_url, extrinsics_file="extrinsics_params.json")

  Or, you can use a 4x4 matrix

    .. code:: python

        ext1 = np.eye(4) # some 4x4 matrix
        source = open_source(source_url, extrinsics=[ext1, ext2, ...])


Deprecated ``build_sensor_config``
++++++++++++++++++++++++++++++++++

The function ``ouster.sdk.sensor.util.build_sensor_config`` has been deprecated and will be removed in the future.

**Replacement:**
Manually create configurations or use ``SensorScanSource`` or ``SensorPacketSource`` for equivalent functionality.

.. code:: python

    # New code:
    from ouster.sdk.sensor import SensorScanSource

    source = SensorScanSource(
        sensor_uri,
        lidar_port=7502, # default None, uses port on lidar configuration
        imu_port=7503,   # default None, uses port on lidar configuration
        do_not_reinitialize=False,
        no_auto_udp_dest=False
    )

Deprecated ``ScanBatcher`` Constructor
++++++++++++++++++++++++++++++++++++++

The constructor ``ScanBatcher::ScanBatcher(size_t w, const sensor::packet_format& pf)`` has been deprecated.

**Replacement:**
Use ``ScanBatcher::ScanBatcher(const sensor_info&)`` instead for better compatibility.


Deprecated ``parse_config``
++++++++++++++++++++++++++++

The function ``ouster::sensor::parse_config`` has been deprecated.

**Replacement:**
Use ``ouster::sensor::parse_and_validate_config`` instead, which provides additional validation for sensor configurations.


Deprecated ``auto_start_flag``
++++++++++++++++++++++++++++++

The attribute ``ouster::sensor::sensor_config::auto_start_flag`` has been deprecated.

**Replacement:**
Use ``ouster::sensor::sensor_config::operating_mode`` instead, which offers better compatibility.


Deprecated ``PointViz`` frame buffer handlers
+++++++++++++++++++++++++++++++++++++++++++++

The function ``push_frame_buffer_handler`` and ``pop_frame_buffer_handler`` in ``PointViz`` will be removed in a future release.

**Replacement:**
For screenshots or screen recording, use one of the following methods:

- ``get_screenshot()``: Captures a screenshot.
- ``save_screenshot()``: Saves a screenshot to a file.
- ``toggle_screen_recording()``: Toggles screen recording functionality.


``OsfFile::version()`` now returns ``ouster::util::version`` instead of ``int``
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

``OsfFile::version()`` now returns a "Semver"-style version in the form of a ``ouster::util::version`` instead of an
``int``.

.. code:: c++

    // old code
    auto file = OsfFile("my_data.osf");
    int version = file.version();  // an integer
    std::cout << "My file version: " << version << std::endl;   // prints 21

    // new code
    auto file = OsfFile("my_data.osf");
    auto version = file.version();  // a struct
    std::cout << "My file version: " << version.simple_version_string() << std::endl;  // prints 2.1.0


Refer to the Ouster SDK documentation for in-depth guidance on each feature and its usage.
