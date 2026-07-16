Using the API
=============

We already have covered how Ouster SDK provides APIs to record data from live sensors and to read from data files in the :doc:`previous section </features/consumption/using-the-api>`.
In this section, we will demonstrate how to write the data that was read into various file formats using the API.

Recording Sensor Data to OSF
-----------------------------

OSF (Open Sensor Format) files provide a more structured approach to data storage, including sensor info and frame-level organization.
It is the recommended format used to store Ouster sensor data. 

Instead of saving raw packets, one can batch packets from a live sensor into LidarFrame objects first, and then saves those ``LidarFrame`` objects to an .osf file. This is useful if you want to store processed, ready-to-use frames.

An API for writing to the OSF file format is also exposed. This is most often used for writing
frames and sensor info, possibly with a reduced number of fields in order to save data.

Let's walkthough an example of recording data from a live sensor and storing it into an OSF file.

.. rubric:: **Imports**

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/playback_osf.py
         :language: python
         :start-after: [doc-stag-osf-write-imports]
         :end-before: [doc-etag-osf-write-imports]
         :class: doc-snippet scroll-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/osf.py>`__
         :dedent: 4
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/record_sensor.cpp
         :language: cpp
         :start-after: [doc-stag-osf-write-imports]
         :end-before: [doc-etag-osf-write-imports]
         :class: doc-snippet scroll-snippet
         :caption: `View on GitHub <|github-src|_snippets/cpp/record_sensor.cpp>`__
         :dedent: 0

.. rubric:: **Setup**

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/playback_osf.py
         :language: python
         :start-after: [doc-stag-osf-write-setup]
         :end-before: [doc-etag-osf-write-setup]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/osf.py>`__
         :dedent: 4
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/record_sensor.cpp
         :language: cpp
         :start-after: [doc-stag-osf-write-setup]
         :end-before: [doc-etag-osf-write-setup]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|_snippets/cpp/record_sensor.cpp>`__
         :dedent: 4

.. rubric:: **Write to OSF file**

A general scheme of writing frames to the OSF with ``Writer``:

1. Create ``osf.Writer`` with the output file name, lidar metadata(s) (``ouster.sdk.core.SensorInfo``) and optionally the desired output frame fields.
2. Use the writer's ``save`` function ``writer.save(index, frame)`` to encode the ``LidarFrame`` into the
   underlying message buffer for lidar ``index`` and finally push it to disk.
   If you have multiple lidar sensors you can save the frames simultaneously by providing them in an array to ``writer.save``.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/playback_osf.py
         :language: python
         :start-after: [doc-stag-osf-write]
         :end-before: [doc-etag-osf-write]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/osf.py>`__
         :dedent: 4
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/record_sensor.cpp
         :language: cpp
         :start-after: [doc-stag-osf-write]
         :end-before: [doc-etag-osf-write]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|_snippets/cpp/record_sensor.cpp>`__
         :dedent: 4


Modifying an OSF File
---------------------

Next, we will look at an example where ``osf.Writer`` is used for saving the available OSF file into Lidar
Frames with a reduced fields. By reduce fields, we mean here that if ``LidarFrame`` has 7 channel
fields, we can keep only 3 and save the disk space and bandwidth during replay.

New field types should be a subset of fields in encoded ``LidarFrame`` so we just assume that ``RANGE``, ``SIGNAL`` and ``REFLECTIVITY`` fields will be present in the input OSF file.
Create Writer with a subset of fields to save (i.e. slicing will happen automatically on write).

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/osf.py
         :start-after: [doc-stag-osf-slice-frames]
         :end-before: [doc-etag-osf-slice-frames]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/osf.py>`__
         :dedent: 4
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude::  _snippets/cpp/record_sensor.cpp
         :language: cpp
         :start-after: [doc-stag-osf-slice-frames]
         :end-before:  [doc-etag-osf-slice-frames]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/osf_writer_example.cpp>`__
         :dedent: 4

Creating an OSF File with custom LidarFrame
------------------------------------------

Below you can see an example which creates a frame and writes it to an OSF File using the Writer API:

.. rubric:: **Imports**

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/playback_osf.py
         :language: python
         :start-after: [doc-stag-osf-create-imports]
         :end-before: [doc-etag-osf-create-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/osf.py>`__
         :dedent: 4
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/osf_writer_example.cpp
         :language: cpp
         :start-after: [doc-stag-osf-create-imports]
         :end-before:  [doc-etag-osf-create-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/osf_writer_example.cpp>`__
         :dedent: 0
         
.. rubric:: **Create LidarFrame and save**

The LidarFrame constructor has an overload taking — width (columns per frame), and height (pixels per column).
The number of columns per packet is set to the default DEFAULT_COLUMNS_PER_PACKET which is 16.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/playback_osf.py
         :language: python
         :start-after: [doc-stag-osf-create]
         :end-before: [doc-etag-osf-create]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/osf.py>`__
         :dedent: 4
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/osf_writer_example.cpp
         :language: cpp
         :start-after: [doc-stag-osf-create]
         :end-before:  [doc-etag-osf-create]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/osf_writer_example.cpp>`__
         :dedent: 4

Playback
---------

Both PCAP and OSF files can be played back using the ``open_source`` helper, which provides a ``FrameSetSource`` for iterating through ``FrameSet`` objects.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: ../consumption/_snippets/python/stream_multi_source.py
         :language: python
         :start-after: [doc-stag-multi-opensource-file]
         :end-before: [doc-etag-multi-opensource-file]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|_snippets/python/stream_multi_source.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: ../consumption/_snippets/cpp/stream_multi_source.cpp
         :language: cpp
         :start-after: [doc-stag-multi-opensource-file]
         :end-before: [doc-etag-multi-opensource-file]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|_snippets/cpp/stream_multi_source.cpp>`__
         :dedent: 4

For a detailed guide on reading and iterating over LidarFrame objects from all source types, please see the :doc:`previous section <../consumption/using-the-api>`.

Converting PCAPs to Other Formats
---------------------------------

Ouster SDK provides several examples demonstrating how to convert PCAP files to various formats such as CSV, LAS, and PCD.
These examples can be found in the :doc:`examples </examples/python/conversion>`.