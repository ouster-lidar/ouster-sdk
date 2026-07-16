
LidarFrame
=========

The frame sources deliver :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` objects when you iterate them, so your downstream code can operate on these structured
frame arrays without dealing with raw packets.

The :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` class aggregates all channel fields (range, reflectivity, signal, near‑IR, etc.)
plus metadata (timestamps, frame IDs, status words) for one full rotation into accessible
fields of the appropriate type.

:ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` also allows for easy projection of the batched data into Cartesian coordinates, producing point clouds.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/lidar_frame_example.py
         :language: python
         :start-after: [doc-stag-lidarframe-imports]
         :end-before: [doc-etag-lidarframe-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/lidar_frame_example.py>`__
         :dedent: 0

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/lidar_frame_example.cpp
         :language: cpp
         :start-after: [doc-stag-lidarframe-imports]
         :end-before: [doc-etag-lidarframe-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/lidar_frame_example.cpp>`__
         :dedent: 0


LidarFrame Constructors
-----------------------

A :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` contains fields of data
specified at its initialization either through a lidar profile or a specific list of fields:

The simplest (and most common) method is to construct one to contain all data coming from your
sensor. You can do this by specifying your sensor's ``sensor_info`` as shown in the snippets below:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/lidar_frame_example.py
         :language: python
         :start-after: [doc-stag-lidarframe-sensorinfo-constructor]
         :end-before: [doc-etag-lidarframe-sensorinfo-constructor]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/lidar_frame_example.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/lidar_frame_example.cpp
         :language: cpp
         :start-after: [doc-stag-lidarframe-sensorinfo-constructor]
         :end-before: [doc-etag-lidarframe-sensorinfo-constructor]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/lidar_frame_example.cpp>`__
         :dedent: 4


But suppose you don't care about some of the data, such as the ambient and signal fields. You can
also specify to your :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` to only batch the relevant fields like so:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/lidar_frame_example.py
         :language: python
         :start-after: [doc-stag-lidarframe-reduced-slots]
         :end-before: [doc-etag-lidarframe-reduced-slots]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/lidar_frame_example.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/lidar_frame_example.cpp
         :language: cpp
         :start-after: [doc-stag-lidarframe-reduced-slots]
         :end-before: [doc-etag-lidarframe-reduced-slots]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/lidar_frame_example.cpp>`__
         :dedent: 4


Accessing LidarFrame fields
--------------------------

Since each :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` corresponds to a single frame (and is batched accordingly), you can access
the ``frame_id`` simply with:


.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/lidar_frame_example.py
         :language: python
         :start-after: [doc-stag-profile-frameid]
         :end-before: [doc-etag-profile-frameid]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/lidar_frame_example.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/lidar_frame_example.cpp
         :language: cpp
         :start-after: [doc-stag-profile-frameid]
         :end-before: [doc-etag-profile-frameid]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/lidar_frame_example.cpp>`__
         :dedent: 4

In addition to ``frame_id`` and the fields specified at initialization, a ``LidarFrame`` also
contains the column header information: ``timestamp``, ``status``, ``measurement_id``. These are
aggregated from each measurement block into W-element arrays, which are represented as an
``Eigen::Array`` and a ``numpy.ndarray`` in C++ and Python respectively. Note that if you
set the sensor configuration parameter ``azimuth_window`` to something less than the full width, the
values in the header outside the azimuth window will be 0'd out accordingly.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/lidar_frame_example.py
         :language: python
         :start-after: [doc-stag-lidarframe-headers]
         :end-before: [doc-etag-lidarframe-headers]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/lidar_frame_example.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/lidar_frame_example.cpp
         :language: cpp
         :start-after: [doc-stag-lidarframe-cpp-headers]
         :end-before: [doc-etag-lidarframe-cpp-headers]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/lidar_frame_example.cpp>`__
         :dedent: 4

For any field contained by a ``LidarFrame``, you can access that field in the following way:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/lidar_frame_example.py
         :language: python
         :start-after: [doc-stag-lidarframe-fields]
         :end-before: [doc-etag-lidarframe-fields]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/lidar_frame_example.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/lidar_frame_example.cpp
         :language: cpp
         :start-after: [doc-stag-lidarframe-cpp-fields]
         :end-before: [doc-etag-lidarframe-cpp-fields]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/lidar_frame_example.cpp>`__
         :dedent: 4

For a more in-depth overview of accessing fields, read :doc:`Field documentation<field>`.


Finally, the fields of an existing :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` can be found by accessing the ``fields`` of the
frame through an iterator:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/pcap.py
         :language: python
         :start-after: [doc-stag-pcap-query-frame]
         :end-before: [doc-etag-pcap-query-frame]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/pcap.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/lidar_frame_example.cpp
         :language: cpp
         :start-after: [doc-stag-pcap-query-frame]
         :end-before: [doc-etag-pcap-query-frame]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/lidar_frame_example.cpp>`__
         :dedent: 4



.. note::

    The units of a particular field from a :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` are consistent even
    when you use lidar profiles which scale the returned data from the sensor.
    This is because :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` will reverse the scaling for you when
    parsing. For example, the RANGE field on a :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` constructed with the
    low data rate profile will be in millimeters even though the return from
    the sensor is given in 8mm increments.


Running the above code on a sample :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` will give you output that looks like:

.. include:: /examples/python/lidar-frame.rst
    :start-after: [start-query-frame-result]
    :end-before: [end-query-frame-result]

Now that we know how to create the :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` and access its contents, let's see what
we can do with it!

.. _custom-fields:

Adding custom fields to a LidarFrame
-----------------------------------

It's possible to add custom fields to a :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>`. This is especially useful if you
want to add custom data to an OSF file for visualization purposes. Like the standard fields normally found in a
:ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>`, custom fields also make use of ``Eigen::Array`` and ``numpy.ndarray`` in C++ and Python, respectively.


.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/lidar_frame.py
         :language: python
         :start-after: [doc-stag-python-frame-add-field]
         :end-before: [doc-etag-python-frame-add-field]
         :class: doc-snippet
         :dedent: 4
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/lidar_frame.py>`__


   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/lidar_frame_example.cpp
         :language: cpp
         :start-after: [doc-stag-cpp-frame-add-field]
         :end-before: [doc-etag-cpp-frame-add-field]
         :dedent:
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/lidar_frame_example.cpp>`__


.. note::
    
   Fields can also be removed using the ``del_field`` method. This can be useful for removing unneeded data, thereby
   saving space when saving frames to an OSF file.


Populating LidarFrames
---------------------

This reference has covered how to create a ``LidarFrame``, and how to access its contents. But in order for ``LidarFrames`` to be useful, we need a way to populate them with packet
data! The recommended approach in both Python and C++ is to use a :ouster:class:`FrameSetSource <py=ouster.sdk.core.FrameSetSource|cpp=ouster::sdk::core::FrameSetSource>`, which handles
both sampling, used in :ref:`ex-visualization-with-matplotlib`, and streaming, used in
:ref:`ex-stream`.

Under the hood, a ``FrameSetSource`` batches packets into ``LidarFrames`` for you. If you already have packets from your own
:ouster:class:`PacketSource <py=ouster.sdk.core.PacketSource|cpp=ouster::sdk::core::PacketSource>` and want to batch them yourself,
both languages also expose a :ouster:class:`FrameBatcher <py=ouster.sdk.core.FrameBatcher|cpp=ouster::sdk::core::FrameBatcher>` class for
that purpose. To get a feel for how to use it, we recommend
reading `this example on GitHub
<https://github.com/ouster-lidar/ouster-sdk/blob/master/examples/client_packet_example.cpp#L81>`_.

FrameBatcher
-----------

The SDK's packet-based :ouster:class:`FrameSetSource <py=ouster.sdk.core.FrameSetSource|cpp=ouster::sdk::core::FrameSetSource>`
implementations use the :ouster:class:`FrameBatcher <py=ouster.sdk.core.FrameBatcher|cpp=ouster::sdk::core::FrameBatcher>`
class to produce :ouster:class:`LidarFrames <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` from UDP
packets. This section describes how the packet batching process works, as well as its guarantees and limitations.

Because Ouster sensors transmit data via UDP, the network or host operating system may re-order or drop packets. The
:ouster:class:`FrameBatcher <py=ouster.sdk.core.FrameBatcher|cpp=ouster::sdk::core::FrameBatcher>` implementation handles
these issues to minimize data loss and latency. It does this by keeping track of which packets have been received for
each frame, and only producing a ``LidarFrame`` when it has received all the necessary packets for that frame. If a packet
from a future frame arrives before all packets from the current frame have been received, the batcher will cache packets
until it receives all packets for the current frame, or until the cache is full, at which point it will finalize the
current frame even though it is incomplete. Fields and headers corresponding to missing packets in the finalized frame
will contain zeros.

The cache size is configurable, giving the user a tradeoff between latency and completeness of frames when packets are
dropped or arrive out of order.

Guarantees & Limitations
^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: FrameBatcher guarantees
   :header-rows: 1

   * - Guarantee
     - Explanation
   * - ``LidarFrame`` frame IDs will increase until roll-over or sensor re-initialization
     - The batcher will produce frames with an increasing frame ID, allowing for roll-over when using standard packet headers (i.e. non-FUSA,) and sensor re-initialization. Otherwise, the batcher will discard any packet from a prior frame. Sensor re-initialization will set the sensor's internal frame counter back to 0, so the batcher will produce new frames with frame IDs starting over at 0 when this occurs.
   * - The batcher caches out-of-order packets from a frame with a greater frame ID than the current frame rather than dropping them
     - The batcher will drop packets from frames it has already finalized, but will cache out-of-order packets from a frame with a greater frame ID until it can produce a complete frame, or until the cache is full. If the cache is full, the new packet will be cached but the batcher will finalize the current frame even though it is incomplete.
   * - The batcher will preserve the contents of custom fields in the provided ``LidarFrame``
     - If you provide a non-empty :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` to the :ouster:func:`batch <py=ouster.sdk.core.FrameBatcher.batch|cpp=ouster::sdk::core::FrameBatcher::batch>`, the batcher will preserve the contents of any custom fields in the frame when producing new frames. This allows you to add custom fields to the frame and have them automatically populated in each produced frame.


.. list-table:: FrameBatcher limitations
   :header-rows: 1

   * - Limitation
     - Explanation
   * - Frames provided to/from the batcher are invalid until finalized
     - The batcher finalizes frames by zeroing portions of headers and fields for which data is missing. Until the FrameBatcher finalizes a frame, (i.e. :ouster:func:`batch <py=ouster.sdk.core.FrameBatcher.batch|cpp=ouster::sdk::core::FrameBatcher::batch>` returns true,) using it results in undefined behavior.
   * - The batcher may not finalize incomplete frames
     - If the user adds no more packets to the batcher, the batcher will not finalize the current frame if it is incomplete.
   * - If the batcher cache fills up with out-of-order packets, it will finalize an incomplete frame
     - If the batcher receives too many out-of-order packets, it will finalize the current frame even though it is incomplete. The batcher will fill missing fields and headers in the finalized frame with zeros.
   * - Dropped packets will delay the current frame
     - If the network or host OS loses a packet for the current frame, the batcher will wait to finalize the current frame until its cache has filled. A number of factors dictate how long this takes, such as the sensor's packet rate, whether IMU and zone monitor are active, and azimuth window.
   * - Out-of-order packets may cause dropped frames
     - If, having just finished frame N, the batcher receives a packet for frame N+2, it will skip frame N+1.
   * - The batcher does not handle changes in sensor configuration
     - Adding packets to a FrameBatcher instance after changing the sensor configuration is unsupported and results in undefined behavior.
   * - The batcher may accept packets from different sensors
     - Adding packets to the same FrameBatcher with different sensors is unsupported and results in undefined behavior.
