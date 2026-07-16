FrameSetSource
==============

Data consumption in the Ouster SDK is built on two core abstractions:

* **PacketSource**: A low-level iterator that yields raw :ouster:class:`LidarPacket <py=ouster.sdk.core.LidarPacket|cpp=ouster::sdk::core::LidarPacket>` and :ouster:class:`ImuPacket <py=ouster.sdk.core.ImuPacket|cpp=ouster::sdk::core::ImuPacket>` objects as they are read from a sensor or file.
* **FrameSetSource**: A high-level iterator that consumes a :ouster:class:`PacketSource <py=ouster.sdk.core.PacketSource|cpp=ouster::sdk::core::PacketSource>`, batches the packets into complete frames, and yields :ouster:class:`FrameSet <py=ouster.sdk.core.FrameSet|cpp=ouster::sdk::core::FrameSet>` objects.

By default, a :ouster:class:`FrameSet <py=ouster.sdk.core.FrameSet|cpp=ouster::sdk::core::FrameSet>` is a container (like a list) that holds one
slot per sensor in the source for a given time slice. Each slot **may** hold a
:ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` for that sensor, but depending on collation a slot may be empty
if no frame was available for that sensor in the given time slice.
For a single sensor source, the :ouster:class:`FrameSet <py=ouster.sdk.core.FrameSet|cpp=ouster::sdk::core::FrameSet>` will simply contain one :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` object.

.. note::

   How a ``FrameSet`` is populated depends on whether the source is collated. By default (``collate=True``),
   each ``FrameSet`` holds one ``LidarFrame`` per sensor for a given time slice. An *uncollated* source instead
   yields a ``FrameSet`` containing a single frame from a single sensor per iteration, cycling between sensors.
   See :doc:`collation` for details.

For almost all applications, you should use a :ouster:class:`FrameSetSource <py=ouster.sdk.core.FrameSetSource|cpp=ouster::sdk::core::FrameSetSource>`.

open_source Helper (Recommended)
--------------------------------

The easiest and most common way to get a :ouster:class:`FrameSetSource <py=ouster.sdk.core.FrameSetSource|cpp=ouster::sdk::core::FrameSetSource>` 
is to use the :ouster:func:`open_source <py=ouster.sdk.open_source|cpp=ouster::sdk::open_source>` helper function.
This function provides a unified interface that automatically detects your data source and instantiates the correct :ouster:class:`FrameSetSource <py=ouster.sdk.core.FrameSetSource|cpp=ouster::sdk::core::FrameSetSource>` implementation for you.

It can handle:

* Live sensors (by hostname or IP)
* .pcap files (with an associated .json metadata file)
* .osf files
* .bag and .mcap files (Python only)

See :ref:`open-source-parameters` in :doc:`using-the-api` for a full description of its usage and parameters.

.. _get-sensor-info:

Obtaining Sensor Info
---------------------

Ouster sensors require sensor info to interpret the readings of the sensor. Represented by the object
:ouster:class:`SensorInfo <py=ouster.sdk.core.SensorInfo|cpp=ouster::sdk::core::SensorInfo>`, metadata fields include configuration parameters such as ``lidar_mode`` and
sensor intrinsics like ``beam_azimuth_angles``.

When you work with a sensor, the client will automatically fetch the metadata. Note that, recorded
``pcaps``, however, must always be accompanied by a ``json`` file containing the metadata of the
sensor as it was when the data was recorded.

Every ``FrameSetSource`` holds a reference to the sensor metadata, which has crucial information that is important when
processing the individual frames. A user can access the metadata through the
``sensor_info`` property of a ``FrameSetSource`` object:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/core.py
         :language: python
         :start-after: [doc-stag-fetch-sensor-info]
         :end-before: [doc-etag-fetch-sensor-info]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/core.py>`__
         :dedent: 4
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/fetch_sensor_info.cpp
         :language: cpp
         :start-after: [doc-stag-fetch-sensor-info]
         :end-before: [doc-etag-fetch-sensor-info]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/fetch_sensor_info.cpp>`__
         :dedent: 4

.. note::
   The ``sensor_info`` property returns a list of ``SensorInfo`` objects, even for single-sensor sources.
   This maintains consistency between single and multi-sensor data sources. Most visualization functions
   accept this list directly, supporting both single and multi-sensor sources automatically. Use ``[0]``
   to access the first sensor's metadata.

Specific FrameSetSource Implementations
---------------------------------------

While open_source is best for most cases, you can instantiate a specific :ouster:class:`FrameSetSource <py=ouster.sdk.core.FrameSetSource|cpp=ouster::sdk::core::FrameSetSource>` class if you need granular control over its parameters.

SensorFrameSetSource
^^^^^^^^^^^^^^^^^^^^^

Use SensorFrameSetSource :ouster:class:`FrameSetSource <py=ouster.sdk.sensor.SensorFrameSetSource|cpp=ouster::sdk::sensor::SensorFrameSetSource>` to connect directly to one or more live sensors.
It manages the network connection, fetches metadata, and batches incoming UDP packets into frames.

It owns the packet capture thread, collates UDP packets into :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` frames, and yields one :ouster:class:`FrameSet <py=ouster.sdk.core.FrameSet|cpp=ouster::sdk::core::FrameSet>` per iteration.

**When to use it**

* You need explicit control over queue depth, per-sensor timeouts, soft-ID validation, or field
  filtering.
* You want completed frames rather than packets, but still need low latency for online processing.

First, let's add the necessary imports to work with the API.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/core.py
         :language: python
         :start-after: [doc-stag-config-imports]
         :end-before: [doc-etag-config-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/core.py>`__
         :dedent: 0
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/config_example.cpp
         :language: cpp
         :start-after: [doc-stag-cpp-config-imports]
         :end-before: [doc-etag-cpp-config-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/config_example.cpp>`__
         :dedent: 0


.. rubric:: Single sensor example

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/stream_single_source.py
         :language: python
         :start-after: [doc-stag-single-sensorframesetsource]
         :end-before: [doc-etag-single-sensorframesetsource]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/stream_single_source.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/stream_single_source.cpp
         :language: cpp
         :start-after: [doc-stag-single-sensorframesetsource]
         :end-before: [doc-etag-single-sensorframesetsource]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/stream_single_source.cpp>`__
         :dedent: 4

.. rubric:: Multiple sensors example

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/stream_multi_source.py
         :language: python
         :start-after: [doc-stag-multi-sensorframesetsource]
         :end-before: [doc-etag-multi-sensorframesetsource]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/stream_multi_source.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/stream_multi_source.cpp
         :language: cpp
         :start-after: [doc-stag-multi-sensorframesetsource]
         :end-before: [doc-etag-multi-sensorframesetsource]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/stream_multi_source.cpp>`__
         :dedent: 4

PcapFrameSetSource
^^^^^^^^^^^^^^^^^^

Use :ouster:class:`PcapFrameSetSource <py=ouster.sdk.pcap.PcapFrameSetSource|cpp=ouster::sdk::pcap::PcapFrameSetSource>` to read from a .pcap file and automatically batch it into complete frames.
This is the simplest, high-level approach if your goal is to work with :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` objects.

A `.pcap` file only contains raw network packets. To interpret this data, :ouster:class:`PcapFrameSetSource <py=ouster.sdk.pcap.PcapFrameSetSource|cpp=ouster::sdk::pcap::PcapFrameSetSource>` must be provided with the sensor's ``.json``
metadata file via the ``meta`` parameter.

Internally, :ouster:class:`PcapFrameSetSource <py=ouster.sdk.pcap.PcapFrameSetSource|cpp=ouster::sdk::pcap::PcapFrameSetSource>` builds upon the packet-level tools described in :ref:`low-level-access` below.

.. rubric:: Imports

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/stream_single_source.py
         :language: python
         :start-after: [doc-stag-pcapframesetsource-imports]
         :end-before: [doc-etag-pcapframesetsource-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/stream_single_source.py>`__
         :dedent: 0

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/stream_single_source.cpp
         :language: cpp
         :start-after: [doc-stag-pcapframesetsource-imports]
         :end-before: [doc-etag-pcapframesetsource-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/stream_single_source.cpp>`__
         :dedent: 0

.. rubric:: Using PcapFrameSetSource to read source

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/stream_single_source.py
         :language: python
         :start-after: [doc-stag-pcapframesetsource-metadata]
         :end-before: [doc-etag-pcapframesetsource-metadata]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/stream_single_source.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/stream_single_source.cpp
         :language: cpp
         :start-after: [doc-stag-pcapframesetsource-metadata]
         :end-before: [doc-etag-pcapframesetsource-metadata]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/stream_single_source.cpp>`__
         :dedent: 4


OsfFrameSetSource
^^^^^^^^^^^^^^^^^

Use :ouster:class:`OsfFrameSetSource <py=ouster.sdk.osf.OsfFrameSetSource|cpp=ouster::sdk::osf::OsfFrameSetSource>` to read from an .osf file.
OSF (Open Sensor Format) is a structured format that conveniently stores the sensor metadata directly within the file, so no separate .json file is needed.

.. rubric:: Imports

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/osf.py
         :language: python
         :start-after: [doc-stag-osf-reader-metadata-imports]
         :end-before: [doc-etag-osf-reader-metadata-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/osf.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/osf_reader_example.cpp
         :language: cpp
         :start-after: [doc-stag-osf-reader-metadata-imports]
         :end-before: [doc-etag-osf-reader-metadata-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/osf_reader_example.cpp>`__
         :dedent: 0

.. rubric:: Using OsfFrameSetSource to read source

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/osf.py
         :language: python
         :start-after: [doc-stag-osf-read-frames]
         :end-before: [doc-etag-osf-read-frames]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/osf.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/stream_single_source.cpp
         :language: cpp
         :start-after: [doc-stag-osf-read-frames]
         :end-before: [doc-etag-osf-read-frames]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/stream_single_source.cpp>`__
         :dedent: 4

This FrameSetSource implementation uses an :ouster:class:`osf.Reader <py=ouster.sdk.osf.Reader|cpp=ouster::sdk::osf::Reader>` internally to read and decode frame messages from the file.

BagFrameSetSource (Python Only)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use :ouster:class:`BagFrameSetSource <py=ouster.sdk.bag.bag_frame_set_source>` to read ROS `.bag` or `.mcap` files. This Python-only class wraps BagPacketSource, which reads ROS messages from the bag and converts them into LidarPacket objects. It then batches these packets into FrameSet objects.

Metadata can be provided via the meta parameter or, in many cases, automatically detected from metadata topics within the bag file.

.. _low-level-access:

Low-Level Access
----------------

Below the ``FrameSetSource`` abstraction, the SDK exposes lower-level tools for working directly with
OSF messages and raw PCAP packets. Reach for these only when the high-level sources above do not give
you the control you need.

osf.Reader
^^^^^^^^^^

You may notice the :ouster:class:`osf.Reader <py=ouster.sdk.osf.Reader|cpp=ouster::sdk::osf::Reader>` class. It's important to understand that ``osf.Reader`` is not a FrameSetSource.

* **OsfFrameSetSource (High-Level)**: An iterator that automatically finds lidar streams, decodes messages, and yields :ouster:class:`FrameSet <py=ouster.sdk.core.FrameSet|cpp=ouster::sdk::core::FrameSet>` objects. Use this for reading frames.

* **osf.Reader (Low-Level)**: The base Reader interface that gets info about start/end_ts, reads and decodes all metadata entries, get access to chunks and messages of the OSF file but it does not automatically decode them into frames.

**When to use osf.Reader directly:**

* Read raw messages or custom metadata from an OSF file.
* Inspect the file's sensor info store without reading all frames.
* Analyze the file's low-level chunk structure or statistics.

A common use for :ouster:class:`osf.Reader <py=ouster.sdk.osf.Reader|cpp=ouster::sdk::osf::Reader>` is to quickly access the sensor info stored in the file without iterating through the frames.

.. _get-sensors-info-with-osf-reader:

Get Sensors Info with ``osf.Reader``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Sensors information is stored as :ouster:class:`osf.LidarSensor <py=ouster.sdk.osf.LidarSensor|cpp=ouster::sdk::osf::LidarSensor>` metadata entry and can be read with the ``reader.meta_store.find()`` function that returns all metadata entry of the specified type (in our case it's of type ``osf.LidarSensor``):

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/osf.py
         :language: python
         :start-after: [doc-stag-osf-reader-metadata]
         :end-before: [doc-etag-osf-reader-metadata]
         :class: doc-snippet scroll-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/osf.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/osf_reader_example.cpp
         :language: cpp
         :start-after: [doc-stag-osf-reader-metadata]
         :end-before: [doc-etag-osf-reader-metadata]
         :class: doc-snippet scroll-snippet
         :caption: `View on GitHub <|github-src|examples/osf_reader_example.cpp>`__
         :dedent: 4

Reading frames with ``osf.Reader``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The code below demonstrates reading :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` from an OSF file using the low-level :ouster:class:`osf.Reader <py=ouster.sdk.osf.Reader|cpp=ouster::sdk::osf::Reader>` class.


.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/osf.py
         :language: python
         :start-after: [doc-stag-osf-reader-messages]
         :end-before: [doc-etag-osf-reader-messages]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/osf.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/osf_reader_example.cpp
         :language: cpp
         :start-after: [doc-stag-osf-read-cpp]
         :end-before: [doc-etag-osf-read-cpp]
         :class: doc-snippet scroll-snippet
         :caption: `View on GitHub <|github-src|examples/osf_reader_example.cpp>`__
         :dedent: 4

Pcap Packet Replay
^^^^^^^^^^^^^^^^^^

If you need to operate on individual packets rather than full frames, you can use the SDK's packet-level tools.
This is useful for experimenting with the SDK APIs without a live sensor or for building custom processing pipelines.

Use :ouster:class:`PcapPacketSource <py=ouster.sdk.pcap.PcapPacketSource|cpp=ouster::sdk::pcap::PcapPacketSource>` to replay packets from a PCAP file.
It yields fully typed :ouster:class:`LidarPacket <py=ouster.sdk.core.LidarPacket|cpp=ouster::sdk::core::LidarPacket>`
and :ouster:class:`ImuPacket <py=ouster.sdk.core.ImuPacket|cpp=ouster::sdk::core::ImuPacket>` objects, with each item carrying the originating sensor index alongside the packet itself.

Example: iterate over typed packets from a PCAP file.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/pcap_packet_source.py
         :language: python
         :start-after: [doc-stag-pcap-packet-source-python]
         :end-before: [doc-etag-pcap-packet-source-python]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/pcap_packet_source.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/pcap_packet_source.cpp
         :language: cpp
         :start-after: [doc-stag-pcap-packet-source-cpp]
         :end-before: [doc-etag-pcap-packet-source-cpp]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/pcap_packet_source.cpp>`__
         :dedent: 4


Refer to the `Ouster SDK on GitHub <https://github.com/ouster-lidar/ouster-sdk>`__ codebase or the `Python API reference <../../python/api_python/ouster.sdk.html>`__ for details beyond the high-level API.
