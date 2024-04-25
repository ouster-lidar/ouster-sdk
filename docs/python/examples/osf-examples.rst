===================
OSF Python Examples
===================

.. contents::
   :local:
   :depth: 3

Ouster Python API for OSF
^^^^^^^^^^^^^^^^^^^^^^^^^

Python OSF Reader/Writer API is a Python binding to the ``C++`` OSF Reader/Writer implementation
which means that all reading and writing operations works at native speeds.


All examples below assume that a user has an ``osf_file`` variable with a path to an OSF file and
``ouster.osf`` package is imported:

.. code::

    import ouster.osf as osf
    
    osf_file = 'path/to/osf_file.osf'

You can use ``ouster-cli source .... save`` commands to generate a test OSF file to test any of the examples.

Every example is wrapped into a CLI and available for quick tests by running 
``python3 -m ouster.sdk.examples.osf <OSF_FILE.osf> <EXAMPLE_NAME>``:

.. code:: bash

    $ python3 -m ouster.sdk.examples.osf --help

    usage: osf.py [-h] [--scan-num SCAN_NUM] OSF EXAMPLE

    Ouster Python SDK OSF examples. The EXAMPLE must be one of:
      read-scans
      read-messages
      split-scans
      slice-scans
      get-lidar-streams
      get-sensors-info
      check-layout

For example to execute the ``get-lidar-streams`` example you can run:

.. code:: bash
    
    $ python3 -m ouster.sdk.examples.osf <OSF_FILE.osf> get-lidar-streams


Read Lidar Scans with ``osf.Scans``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``osf.Scans()`` interface is the simplest way to get all ``LidarScan`` objects for the first sensor
that was found in an OSF (majority of our test data uses only a single sensor recordings):

.. literalinclude:: /../python/src/ouster/sdk/examples/osf.py
    :start-after: [doc-stag-osf-read-scans]
    :end-before: [doc-etag-osf-read-scans]
    :dedent:

Underneath it looks for available sensor streams, peeks first, creates the ``osf.Reader``, reads the
**messages** and decodes them to ``LidarScan`` objects.

.. admonition:: Note about timestamp ``ts``

    All messages in an OSF are stored with a timestamp so it's an essential part of the stream
    during the read operation. If later you will decide to store the post-processed ``LidarScan``
    back into another OSF it's better to preserve the original ``ts`` which usually came from
    NIC/PCAP/BAG headers. To get ``ts`` along with ``LidarScan`` use ``osf.Scans().withTs()``
    iterator.

Get Sensors Info with ``osf.Reader``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``osf.Reader`` is the base ``Reader`` interface that get info about ``start/end_ts``, reads and
decodes all **metadata entries**, get access to **chunks** and **messages** of the OSF file.

Sensors information is stored as ``osf.LidarSensor`` metadata entry and can be read with the
``reader.meta_store.find()`` function that returns all metadata entry of the specified type (in our
case it's of type ``osf.LidarSensor``):

.. literalinclude:: /../python/src/ouster/sdk/examples/osf.py
    :start-after: [doc-stag-osf-get-sensors-info]
    :end-before: [doc-etag-osf-get-sensors-info]
    :dedent:


Read All Messages with ``osf.Reader``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

With ``osf.Reader``, you can use ``reader.messages()`` iterator to read messages in ``timestamp``
order.

.. literalinclude:: /../python/src/ouster/sdk/examples/osf.py
    :start-after: [doc-stag-osf-read-all-messages]
    :end-before: [doc-etag-osf-read-all-messages]
    :dedent:


Checking Chunks Layout via ``osf.StreamingInfo``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Building on top of an example from above we can check for stream
statistics information from ``osf.StreamingInfo``:

.. literalinclude:: /../python/src/ouster/sdk/examples/osf.py
    :start-after: [doc-stag-osf-check-layout]
    :end-before: [doc-etag-osf-check-layout]
    :dedent:

For more information about ``osf.StreamingInfo`` metadata entry please refer to [RFC 0018]_.


Get Lidar Scan streams info via ``osf.LidarScanStream``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Every message in an OSF belongs to a stream of a particular type (i.e. ``osf.LidarScanStream``,
``osf.LidarImuStream``, etc.). Streams information stored as **metadata entry** within
``osf.Reader.meta_store`` object that can be read and decoded in various ways. Below is an example
of how we can check parameters of an available LidarScan streams (``osf.LidarScanStream``) by
checking the metadata entries:

.. literalinclude:: /../python/src/ouster/sdk/examples/osf.py
    :start-after: [doc-stag-osf-get-lidar-streams]
    :end-before: [doc-etag-osf-get-lidar-streams]
    :dedent:


Write Lidar Scan with sliced fields with ``osf.Writer``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We will look into the ``osf.Writer`` example on the task of re-coding the available OSF file into Lidar
Scans with a reduced fields. By reduce fields we mean here that if LidarScan has ``7`` channel
fields, we can keep only ``3`` and save the disk space and bandwidth during replay.

A general scheme of writing scans to the OSF with Writer:

0. Create ``osf.Writer`` with the output file name, lidar metadata(s) (``ouster.sdk.client.SensorInfo``) and optionally the desired output scan fields.
1. Use the writers's ``save`` function ``writer.save(index, scan)`` to encode the LidarScan ``scan`` into the
   underlying message buffer for lidar ``index`` and finally push it to disk. If you have multiple lidars you can
   save the scans simultaneously by providing them in an array to ``writer.save``.

.. literalinclude:: /../python/src/ouster/sdk/examples/osf.py
    :start-after: [doc-stag-osf-slice-scans]
    :end-before: [doc-etag-osf-slice-scans]
    :dedent:


Split Lidar Scan stream into multiple files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Another example of using ``osf.Writer`` that we will see is the splitting of Lidar Scan stream from
one OSF file into 2 files.

.. literalinclude:: /../python/src/ouster/sdk/examples/osf.py
    :start-after: [doc-stag-osf-split-scans]
    :end-before: [doc-etag-osf-split-scans]
    :dedent:
