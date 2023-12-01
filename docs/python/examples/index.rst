===================
Examples & Concepts
===================

.. _examples-setup:

A loosely connected collection of examples and concepts useful for working with the Ouster Python
SDK. If you are just starting, please see :doc:`/python/quickstart`.

For convenience, and in keeping it with :doc:`quickstart </python/quickstart>` section, we will use
``$SAMPLE_DATA_PCAP_PATH`` and ``$SAMPLE_DATA_JSON_PATH`` for the locations of the sample data pcap
and json. For Python code, ``pcap_path`` and ``json_path`` are taken to be set to those values,
respectively:

.. code:: python

   pcap_path = '<SAMPLE_DATA_PCAP_PATH>'
   metadata_path = '<SAMPLE_DATA_JSON_PATH>'

Similarly, ``$SENSOR_HOSTNAME`` is used for your sensor's hostname.

.. toctree::

   basics-sensor
   udp-packets
   lidar-scan
   record-stream
   visualizations
   conversion


