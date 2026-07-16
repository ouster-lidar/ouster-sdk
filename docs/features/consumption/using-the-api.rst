open_source & open_packet_source
================================

.. note::

   SDK 1.0 introduces breaking changes to the data consumption APIs described here; see :doc:`../../migration/migration-0.16.2-1.0` when upgrading from 0.16.x.

The Ouster SDK provides two primary helper functions for data consumption, :ouster:func:`open_source <py=ouster.sdk.open_source|cpp=ouster::sdk::open_source>`
and :ouster:func:`open_packet_source <py=ouster.sdk.open_source.open_packet_source|cpp=ouster::sdk::open_packet_source>`.
The one you choose depends on your goal:

- To work with complete frames (most common): Use ``open_source`` to get a :ouster:class:`FrameSetSource <py=ouster.sdk.core.FrameSetSource|cpp=ouster::sdk::core::FrameSetSource>` that yields :ouster:class:`FrameSet <py=ouster.sdk.core.FrameSet|cpp=ouster::sdk::core::FrameSet>` objects.

- To work with raw packets (advanced): Use ``open_packet_source`` to get a :ouster:class:`PacketSource <py=ouster.sdk.core.PacketSource|cpp=ouster::sdk::core::PacketSource>` that yields individual :ouster:class:`LidarPacket <py=ouster.sdk.core.LidarPacket|cpp=ouster::sdk::core::LidarPacket>` and :ouster:class:`ImuPacket <py=ouster.sdk.core.ImuPacket|cpp=ouster::sdk::core::ImuPacket>` objects.

Reading Frames from a Live Sensor
----------------------------------

The ``FrameSetSource`` API uses the method name ``open_source`` which provides a uniform entry point for users to handle different source types
using the same API. Currently, the supported source types are live sensor, pcap file, or osf file. The Python API also supports rosbag files.

It detects the IO type from the string, calls the appropriate constructor under the hood and then applies extra options.

For example, to start consuming packets from a single live sensor, you can call ``open_source`` with the hostname of the sensor:

.. rubric:: **Imports**

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/stream_single_source.py
         :language: python
         :start-after: [doc-stag-opensource-imports]
         :end-before: [doc-etag-opensource-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/stream_single_source.py>`__
         :dedent: 0

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/stream_single_source.cpp
         :language: cpp
         :start-after: [doc-stag-opensource-imports]
         :end-before: [doc-etag-opensource-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/stream_single_source.cpp>`__
         :dedent: 0

.. rubric:: **Instantiate open_source**

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/stream_single_source.py
         :language: python
         :start-after: [doc-stag-single-opensource]
         :end-before: [doc-etag-single-opensource]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/stream_single_source.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/stream_single_source.cpp
         :language: cpp
         :start-after: [doc-stag-single-opensource]
         :end-before: [doc-etag-single-opensource]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/stream_single_source.cpp>`__
         :dedent: 4

Or to replay from a valid input file source file such as pcap or osf file, you can call ``open_source`` with the file path:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/stream_single_source.py
         :language: python
         :start-after: [doc-stag-pcap-replay]
         :end-before: [doc-etag-pcap-replay]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/stream_single_source.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/stream_single_source.cpp
         :language: cpp
         :start-after: [doc-stag-pcap-replay]
         :end-before: [doc-etag-pcap-replay]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/stream_single_source.cpp>`__
         :dedent: 8


Load Metadata for Pcap Sources
------------------------------

PCAP (Packet Capture) files are the most common format for recording raw UDP packets from Ouster sensors.
They preserve the original network packets exactly as received.

The ``source`` object returned by ``open_source`` provides access to :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` objects,
regardless of whether the source data comes from a sensor, pcap, or osf file.

For pcap sources, if the metadata file is not in the same folder as the pcap and doesn't have a shared name prefix the above method will
fail.

To load and parse the metadata ourselves we only need to pass the metadata to the method through the optional ``meta`` parameter
and the method will take care of loading it and associating it with the
source object.


.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/stream_single_source.py
         :language: python
         :start-after: [doc-stag-pcap-replay-metadata]
         :end-before: [doc-etag-pcap-replay-metadata]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/stream_single_source.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/stream_single_source.cpp
         :language: cpp
         :start-after: [doc-stag-pcap-replay-metadata]
         :end-before: [doc-etag-pcap-replay-metadata]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/stream_single_source.cpp>`__
         :dedent: 4


.. _open-source-parameters:

``open_source`` Parameters
--------------------------

One can process data from single or multiple sensors by passing a hostname, file path, or a list of sources to the SDK.
The output behavior is primarily controlled by the collate and sensor_idx parameters. The function signature is as follows:


.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         def open_source(
             source_url: Union[str, List[str]],
             collate: bool = True,
             sensor_idx: int = -1,
             *args,
             **kwargs) -> FrameSetSource: ...

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         core::AnyFrameSetSource open_source(
             const std::string& source,
             const std::function<void(FrameSetSourceOptions&)>& options = {},
             bool collate = true,
             int sensor_idx = -1
         );

**collate (default: True)**:
      - ``True``: Yields one synchronized ``FrameSet`` per iteration with one entry per sensor — a "larger" ``FrameSet`` that time-aligns multiple sensors' frames together.
      - ``False``: Yields individual ``FrameSet`` objects as soon as they're available (only recommended for single-sensor use). On multi-sensor sources still yields ``FrameSet`` objects, but they arrive one per sensor with no time alignment.

**sensor_idx (default: -1)**:
      - ``-1``: Keeps every sensor stream in the source, allowing collation.
      - ``>= 0``: Returns only the sensor with the specified index; the ``collate`` flag is ignored.

The output is always a ``FrameSet``, which acts as a container. The parameters determine how many frames this set contains and how they are synchronized.

.. list-table:: ``open_source`` parameters
   :header-rows: 1

   * - Parameter
     - Type
     - Default
     - Purpose
   * - ``source``
     - ``str`` or ``List[str]``
     - required
     - Sensor hostname(s) or file path; all entries must be the same IO type.
   * - ``options``
     - ``FrameSetSourceOptions`` or ``kwargs``
     - unset
     - FrameSetSourceOptions available are listed at :ref:`frame-set-source-options-table`.
   * - ``collate``
     - ``bool``
     - ``True``
     - Returns synchronized ``FrameSet``; ignored when ``sensor_idx >= 0``.
   * - ``sensor_idx``
     - ``int``
     - ``-1``
     - ``-1`` streams every sensor; any ``>= 0`` selects that sensor via given index and bypasses collation.


Example for Single Sensor Source:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/stream_single_source.py
         :language: python
         :start-after: [doc-stag-single-opensource-nocollate]
         :end-before: [doc-etag-single-opensource-nocollate]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/stream_single_source.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/stream_single_source.cpp
         :language: cpp
         :start-after: [doc-stag-single-opensource-nocollate]
         :end-before: [doc-etag-single-opensource-nocollate]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/stream_single_source.cpp>`__
         :dedent: 4

Example for Multi Sensor Source:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/stream_multi_source.py
         :language: python
         :start-after: [doc-stag-multi-opensource-sensor]
         :end-before: [doc-etag-multi-opensource-sensor]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/stream_multi_source.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/stream_multi_source.cpp
         :language: cpp
         :start-after: [doc-stag-multi-opensource-sensor]
         :end-before: [doc-etag-multi-opensource-sensor]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/stream_multi_source.cpp>`__
         :dedent: 4


.. note::

   The PCAP and OSF builders allows opening only one file at a time so ``open_source`` cannot be used for multiple file inputs such as the code shown below.

   .. tab-set::
      :sync-group: api-lang

      .. tab-item:: Python
         :sync: py

         .. literalinclude:: _snippets/python/stream_multi_source.py
            :language: python
            :start-after: [doc-stag-multi-opensource-file]
            :end-before: [doc-etag-multi-opensource-file]
            :class: doc-snippet
            :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/stream_multi_source.py>`__
            :dedent: 4

      .. tab-item:: C++
         :sync: cpp

         .. literalinclude:: _snippets/cpp/stream_multi_source.cpp
            :language: cpp
            :start-after: [doc-stag-multi-opensource-file]
            :end-before: [doc-etag-multi-opensource-file]
            :class: doc-snippet
            :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/stream_multi_source.cpp>`__
            :dedent: 4



.. _frame-set-source-options-table:

``FrameSetSourceOptions`` Parameters
------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 18 18 10 14 40

   * - Option
     - Type
     - Default
     - Applies to
     - Purpose
   * - ``extrinsics_file``
     - ``str``
     - unset
     - PCAP, OSF, live
     - Load per-sensor extrinsics from JSON (matched by serial number).
   * - ``extrinsics``
     - list of 4×4 matrices
     - unset
     - All
     - Override file/default extrinsics with explicit transforms.
   * - ``field_names``
     - ``List[str]`` or ``None``
     - unset
     - All
     - Restrict decoded channels (``None`` keeps defaults, empty list decodes none).
   * - ``soft_id_check``
     - ``bool``
     - ``False``
     - All
     - Accept packets/frames whose serial/init ID differs from metadata.
   * - ``index``
     - ``bool``
     - ``False``
     - File sources
     - Force building an index before streaming.
   * - ``meta``
     - ``List[str]``
     - unset
     - PCAP, OSF
     - Provide explicit metadata files when auto-discovery fails.
   * - ``lidar_port`` / ``imu_port``
     - ``Optional[int]``
     - unset
     - Live sensors
     - Override UDP ports when connecting to hardware.
   * - ``do_not_reinitialize``
     - ``bool``
     - ``False``
     - Live sensors
     - Leave existing sensor settings untouched on connect.
   * - ``no_auto_udp_dest``
     - ``bool``
     - ``False``
     - Live sensors
     - Prevent the SDK from rewriting the sensor’s UDP destination.
   * - ``timeout``
     - ``float`` seconds
     - ``1.0``
     - All (iterators)
     - Per-sensor wait time for frames when iterating.
   * - ``config_timeout``
     - ``float`` seconds
     - ``45.0``
     - Live sensors
     - HTTP/configuration timeout while staging sensor settings.
   * - ``queue_size``
     - ``unsigned int``
     - ``2``
     - Live sensors
     - Cap buffered frames to avoid backlog growth.
   * - ``sensor_info``
     - ``List[SensorInfo]`` / unset
     - unset
     - Live sensors
     - Supply metadata without querying devices.
   * - ``raw_headers`` / ``raw_fields``
     - ``bool``
     - ``False``
     - All
     - Include raw per-column headers or raw pixel words in each ``LidarFrame``.
   * - ``sensor_config``
     - ``List[SensorConfig]`` / unset
     - unset
     - Live sensors
     - Apply staged configuration before streaming.
   * - ``error_handler``
     - ``ouster::core::error_handler_t``
     - ``default_error_handler``
     - All
     - Override SDK error-reporting callbacks.


Reading raw packet data
=======================

While most applications should use a ``FrameSetSource`` to work with complete ``LidarFrame`` objects, the SDK also provides a lower-level API for working directly with raw packets.

The ``open_packet_source`` function is a high-level helper designed to automatically detect the source type and return the correct ``PacketSource`` implementation.

A ``PacketSource`` is an iterator that yields raw, individual ``LidarPacket`` and ``ImuPacket`` objects as they are received from a sensor or read from a file.
This is primarily useful for tasks if your application requires access to raw packet data before it's batched into frames such as
recording data, network replay, or custom, low-level processing.

Valid inputs for ``open_packet_source`` include:

- Live sensor hostnames/IPs
- .pcap files
- .bag files (Python only)

An example of using ``open_packet_source`` to read packets from a live sensor is shown below.:

.. rubric:: **Imports**

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/read_packet.py
         :language: python
         :start-after: [doc-stag-pcap-record-imports]
         :end-before: [doc-etag-pcap-record-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/read_packet.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/read_packet.cpp
         :language: cpp
         :start-after: [doc-stag-pcap-record-imports]
         :end-before: [doc-etag-pcap-record-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/read_packet.cpp>`__
         :dedent: 0


.. rubric:: **Read packet data**

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/read_packet.py
         :language: python
         :start-after: [doc-stag-pcap-record-setup]
         :end-before: [doc-etag-pcap-record-setup]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/read_packet.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/read_packet.cpp
         :language: cpp
         :start-after: [doc-stag-pcap-record-setup]
         :end-before: [doc-etag-pcap-record-setup]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/read_packet.cpp>`__
         :dedent: 4

Frame Iteration
---------------

The ``FrameSetSource`` examines the timestamp of every frame from every sensor and returns a list of frames
that fit within the same time window as a single ``FrameSet``. The number of slots in the ``FrameSet`` is fixed corresponding
to how many sensors are contained in the pcap or OSF file (one entry per sensor).

However, the collation could yield a null value if one or more of the sensors didn’t produce a ``LidarFrame``
object that fits within the time frame of the current ``FrameSet`` or iteration.

Thus, depending on the operation at hand it is critical to check if we got a valid ``LidarFrame`` object
when examining the iteration output of a ``FrameSetSource``.

To display the ``frame_id`` of ``LidarFrame`` objects that belong to the same ``FrameSet`` on the same line, the code needs to be updated to the following:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/stream_multi_source.py
         :language: python
         :start-after: [doc-stag-multi-sensorframesetsource-loop]
         :end-before: [doc-etag-multi-sensorframesetsource-loop]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/stream_multi_source.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/stream_multi_source.cpp
         :language: cpp
         :start-after: [doc-stag-multi-sensorframesetsource-loop]
         :end-before: [doc-etag-multi-sensorframesetsource-loop]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/stream_multi_source.cpp>`__
         :dedent: 4

Selecting one sensor: ``sensor_idx`` vs ``single()``
----------------------------------------------------

``single(n)`` is the underlying operation that selects a single sensor; it returns a ``Singler`` that keeps only the
sensor at index ``n``. ``single()`` must be called on an *uncollated* source, so open with ``collate=False`` first.
The ``sensor_idx`` parameter of ``open_source`` is simply a shortcut that opens the source uncollated and calls ``single(sensor_idx)`` for you, so these two are
equivalent:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/stream_single_source.py
         :language: python
         :start-after: [doc-stag-single-select-equivalent]
         :end-before: [doc-etag-single-select-equivalent]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/stream_single_source.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/stream_single_source.cpp
         :language: cpp
         :start-after: [doc-stag-single-select-equivalent]
         :end-before: [doc-etag-single-select-equivalent]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/stream_single_source.cpp>`__
         :dedent: 4

For a source that contains only one sensor (such as a single live sensor), explicitly selecting ``sensor_idx=0`` above
is equivalent to leaving ``sensor_idx`` at its default of ``-1``: there is only one stream, so each iteration yields a
one-entry ``FrameSet`` either way. The difference only becomes observable with multi-sensor sources, where ``-1`` keeps
all streams (collated) while ``sensor_idx >= 0`` narrows the output to the single sensor at that index.

Either of these two forms yields an iterator over ``FrameSet`` objects, but since the source is now narrowed to one sensor, each
``FrameSet`` holds only a single entry. Inside the loop, ``frame_set`` is that ``FrameSet``: ``frame_set[0]`` grabs the
``LidarFrame`` object, and printing ``frame_set[0].frame_id`` gives the frame number for that sensor.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/stream_single_source.py
         :language: python
         :start-after: [doc-stag-single-opensource-nocollate-loop]
         :end-before: [doc-etag-single-opensource-nocollate-loop]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/stream_single_source.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/stream_single_source.cpp
         :language: cpp
         :start-after: [doc-stag-single-opensource-nocollate-loop]
         :end-before: [doc-etag-single-opensource-nocollate-loop]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/stream_single_source.cpp>`__
         :dedent: 4

Calling ``single()`` on a collated source (the ``open_source`` default) raises an error because collation discards the
per-sensor stream boundaries — there is no way to recover them afterward. Selecting an index the source does not contain
(for example ``single(2)`` on a single-sensor source) also raises.

Closing a source
----------------

When you are finished with a source, you should close it to release its underlying resources. For a live sensor this frees
the UDP sockets bound to the lidar and IMU ports; for pcap and osf replay it releases the open file
handles. This behaves identically regardless of how you chain or single them.

.. note::

   If you do not release a live source, the sensor's UDP ports stay bound until the process exits.
   Attempting to reopen the same sensor before the port is released fails.

FrameSetSources in the SDK generally follow RAII: when they go out of scope or are otherwise destructed, they automatically
release their underlying resources. In C++ you can do this by just letting the object go out of scope when you are finished or
by doing something like storing the object in a smart pointer and resetting it. In Python you can do this by utilizing the source in
a context manager (``with`` block). Both means will ensure the source is closed when you are done with it,
including if any exceptions are thrown.

In Python however, RAII based approaches do not always work so a ``close()`` method is also exposed. When this method is called the
resources from the source will immediately be released, making further interaction with the source prone to errors or failure.
It is recommended you follow up every close by setting the variable holding the source to None (or a new source) to ensure you do not interact with a closed source.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/stream_single_source.py
         :language: python
         :start-after: [doc-stag-single-opensource-close]
         :end-before: [doc-etag-single-opensource-close]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/stream_single_source.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/stream_single_source.cpp
         :language: cpp
         :start-after: [doc-stag-single-opensource-close]
         :end-before: [doc-etag-single-opensource-close]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/stream_single_source.cpp>`__
         :dedent: 4
