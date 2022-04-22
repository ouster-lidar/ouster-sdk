.. _ex-packets:

=====================
Lidar and IMU Packets
=====================


The :py:class:`.PacketSource` is the basic interface for sensor packets. It can be advantageous to
work with packets directly when latency is a concern, or when you wish to examine the data packet by
packet, e.g., if you wish to examine timestamps of packets.

Let's make a :py:class:`.PacketSource` from our sample data using :py:class:`.pcap.Pcap`:

.. code:: python

    with open(metadata_path, 'r') as f:
        metadata = client.SensorInfo(f.read())

    source = pcap.Pcap(pcap_path, metadata)

Now we can read packets from ``source`` with the following code:

.. literalinclude:: /../python/src/ouster/sdk/examples/pcap.py
    :start-after: [doc-stag-pcap-read-packets]
    :end-before: [doc-etag-pcap-read-packets]
    :dedent:

You can try the above code with the :py:func:`~.pcap.pcap_read_packets`:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH read-packets

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH read-packets

