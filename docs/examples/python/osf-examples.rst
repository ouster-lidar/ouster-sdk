.. _ex-osf-python-examples:

Working with OSF
================

Python OSF Reader/Writer API is a Python binding to the ``C++`` OSF Reader/Writer implementation
which means that all reading and writing operations works at native speeds.


You can use ``ouster-cli source .... save`` commands to generate a test OSF file to test any of the examples.

Every example is wrapped into a CLI and available for quick tests by running 
``python3 -m ouster.sdk.examples.osf <OSF_FILE.osf> <EXAMPLE_NAME>``:

.. code:: bash

    $ python3 -m ouster.sdk.examples.osf --help

    usage: osf.py [-h] [--frame-num FRAME_NUM] OSF EXAMPLE

    Ouster Python SDK OSF examples. The EXAMPLE must be one of:
      read-frames
      slice-frames
      get-sensors-info

For example to execute the ``read-frames`` example you can run:

.. code:: bash
    
    $ python3 -m ouster.sdk.examples.osf <OSF_FILE.osf> read-frames




Write Lidar Frame with sliced fields with ``osf.Writer``
--------------------------------------------------------

We will look into the ``osf.Writer`` example on the task of re-coding the available OSF file into Lidar
frames with a reduced fields. By reducing fields we mean here that if LidarFrame has ``7`` channel
fields, we can keep only ``3`` and save the disk space and bandwidth during replay.

.. note:: 

    Update Link after MR 2 merge

See :doc:`/features/consumption/frame_set_sources` for details.

.. code:: bash
    
    $ python3 -m ouster.sdk.examples.osf <OSF_FILE.osf> slice-frames


Get Sensors Info with ``osf.Reader``
------------------------------------

``osf.Reader`` is the base ``Reader`` interface that get info about ``start/end_ts``, reads and
decodes all **metadata entries**, get access to **chunks** and **messages** of the OSF file.

See :doc:`get-sensors-info-with-osf-reader </features/consumption/frame_set_sources>` for details.

.. code:: bash
    
    $ python3 -m ouster.sdk.examples.osf <OSF_FILE.osf> get-sensors-info
