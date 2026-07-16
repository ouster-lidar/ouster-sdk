Using the CLI
=============

We already have covered how to use Ouster CLI for recording data from live sensors and to read from data files (see :ref:`Load sensor source <load-sensor-source>`).
In this section, we will demonstrate how to write the data that was read into various file formats using the CLI.

The snippets below read environment variables such as ``SENSOR_HOSTNAME`` and
``SAMPLE_DATA_OSF_PATH``. Override them before running the commands, or export
them inline on the shell.

Recording Sensor Data to Supported File Formats
-----------------------------------------------

To capture frames from a live sensor into a supported file format, ``ouster-cli`` can be used with the ``save`` command.

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-record-pcap]
   :end-before: [doc-etag-cli-record-pcap]
   :caption: Save frames as pcap files
   :dedent: 0


The CLI’s ``source … save`` pipeline works with any input the source command can open — PCAP, OSF, BAG, MCAP, or a live sensor.
The dispatcher looks at the file extension, and forwards execution to the matching handler.

To capture frames directly into OSF, use the ``save`` command with an ``.osf`` filename. Use ``--overwrite`` optionally to overwrite existing files with the same name.

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-record-osf]
   :end-before: [doc-etag-cli-record-osf]
   :caption: Record live frames to osf output
   :dedent: 0

Use ``--dir`` to specify an output directory. The ``{VALID_INPUT_SOURCE}`` must be set to a live sensor hostname/ IP address or a valid PCAP/ OSF/ BAG/ MCAP file path.

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-record-csv]
   :end-before: [doc-etag-cli-record-csv]
   :caption: Save frames as CSV files
   :dedent: 0

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-record-png]
   :end-before: [doc-etag-cli-record-png]
   :caption: Save frames as PNG heatmaps
   :dedent: 0


These commands run until interrupted (``CTRL+C``). Use ``--split`` to rotate output files at a size
threshold. 


Recording Packet data to PCAP/BAG files
---------------------------------------

When one needs to save raw packets from the source, we can use ``save_raw`` command. This supports saving into PCAP/BAG files.

To record data from a udp port (7502 by default) to a pcap file in the current directory and write
the metadata to a json file with the same name, simply use:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-record-pcap-raw]
   :end-before: [doc-etag-cli-record-pcap-raw]
   :caption: Record packets and metadata output
   :dedent: 0

Similarly, specify ``.bag`` to capture raw packets into a ROS bag:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-record-bag]
   :end-before: [doc-etag-cli-record-bag]
   :caption: Record packets to bag output
   :dedent: 0

The extension you pass determines the output format; using ``.pcap`` preserves the raw UDP packets. A matching
metadata JSON file is written alongside the PCAP.

The ``save_raw`` command also accepts ``--duration`` to stop automatically after the
requested number of seconds. 

Consult ``ouster-cli source <SOURCE> save --help`` for the full option set.



.. note::

   ``ouster-cli`` does not currently expose an option to drop specific channel fields when saving
   to OSF. Use the SDK examples in :doc:`../playback/using-the-api` if you need to thin the field set.


Replaying Data as a Live Sensor (BETA)
--------------------------------------

``sensor_replay`` can replay a PCAP/OSF/BAG/MCAP stream as if it were a live sensor. Point
``ouster-cli`` at a dataset and specify the destination hostname (or interface) for the emulated
sensor packets.

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-sensor-replay]
   :end-before: [doc-etag-cli-sensor-replay]
   :caption: Replay a recording as a virtual sensor
   :dedent: 0

You can view the sensor webpage at http://127.0.0.1:8080. 

.. note::

   This is a **BETA** feature.
   
   This emulator implements a limited subset of the device capabilities and does not fully model the actual sensor behavior.

   The emulator allows the change of the following list of configuration options (All other settings are read-only):

   * UDP Destination Address (udp_dest)
   * UDP Lidar Port (lidar_port)
   * UDP IMU Port (imu_port)
   * Operating Mode (operating_mode)

   **Important**: Applications developed and tested using this emulator should be validated on actual Ouster hardware to ensure correct functionality and performance.


This BETA feature requires Docker when using ``--dockerize``. See help for supported transports and options.

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-sensor-replay-help]
   :end-before: [doc-etag-cli-sensor-replay-help]
   :dedent: 0

