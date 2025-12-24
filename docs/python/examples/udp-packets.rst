.. _ex-packets:

=====================
Lidar and IMU Packets
=====================


The :py:class:`.core.PacketSource` (previously `PacketMultiSource`) is the basic interface for sensor packets 
and returns `Tuple[int, Packet]`. It can be advantageous to work with packets directly when latency is a 
concern, or when you wish to examine the data packet by packet, e.g., if you wish to examine timestamps of packets.

Let's make a :py:class:`.core.PacketSource` from our sample data using :py:class:`.pcap.PcapPacketSource`:

.. code:: python
    
    from ouster.sdk import pcap
    source = pcap.PcapPacketSource(pcap_path)
    for _, packet in source:
        ...  # a list of packets, with (sensor_idx, Packet)


Now we can read packets from ``source`` with the following code:

.. literalinclude:: /../python/src/ouster/sdk/examples/pcap.py
    :start-after: [doc-stag-pcap-read-lidar-packets]
    :end-before: [doc-etag-pcap-read-lidar-packets]
    :dedent:

.. literalinclude:: /../python/src/ouster/sdk/examples/pcap.py
    :start-after: [doc-stag-pcap-read-imu-packets]
    :end-before: [doc-etag-pcap-read-imu-packets]
    :dedent:

You can try the above code with the :py:func:`~.examples.pcap.pcap_read_packets`:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH read-packets

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH read-packets

