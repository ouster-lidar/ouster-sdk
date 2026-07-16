Using the CLI
=============

The ``ouster-cli`` tool provides a command line interface to interact with Ouster sensors and data.

Install the ouster-sdk python package to get access to the ``ouster-cli`` utility. 
See :doc:`/getting-started/installation` for installation instructions.

After installation, to verify installation and view command options, open a terminal and run:

.. code::

    ouster-cli --help

Collecting Metadata
+++++++++++++++++++

Sensor metadata, necessary for interpreting and parsing the pcap data, can be collected from sensors
using:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-get-metadata]
   :end-before: [doc-etag-cli-get-metadata]
   :dedent: 0

This will generate a ``.json`` file named ``{SENSOR_HOSTNAME}.json`` with the metadata inside.
To output it to a differently named file, simply change ``{SENSOR_HOSTNAME}.json`` to
``{FILL_IN_ALTERNATE_NAME}.json``.

You can also print the metadata to screen by removing ``>`` and
everything after it in the command.


.. _load-sensor-source:

Load sensor source
++++++++++++++++++

The ``source`` command selects the sensor or recording that subsequent commands operate on.
Run ``ouster-cli source --help`` to see the available options and subcommands. After the source is
bound, chain whichever consumer you need such as ``stats``, ``viz``, or ``save``.

Inspect a live sensor directly from the terminal:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-source-live]
   :end-before: [doc-etag-cli-source-live]
   :dedent: 0

By default, running ``ouster-cli source {SENSOR HOSTNAME}`` will reconfigure the sensor to
transmit lidar packets to the host running ``ouster-cli`` and reinitialize the sensor.

One can disable the default reinitialization/UDP destination behavior or adjust timeouts before launching the
visualizer as mentioned below:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-source-live-viz]
   :end-before: [doc-etag-cli-source-live-viz]
   :dedent: 0

Remember that you can use ``--help`` with any ``ouster-cli`` subcommand, regardless how far down the
menu tree you are.

.. admonition:: Ubuntu UFW Firewall may cause: ``No packets received within 1.0s``

    On some Ubuntu setups we've observed the situations when everything is configured properly so
    that:

    - sensor is seen on the network and its Web page can be reached
    - sensor destination IP is set to the IP of the computer where data is expected
    - sensor lidar port is known (i.e. default ``7502``, or some others)
    - sensor is in ``RUNNING`` state
    - sensor lidar packets traffic is seen on the expected machine and can be recorded with
      ``tcpdump -w`` command to a pcap file (or ``Wireshark`` tools)
    - CLI command ``ouster-cli source <SENSOR HOSTNAME> {info,config}`` are working properly
    - Viz ``ouster-cli source <PCAP FILE> viz`` from the ``tcpdump`` recorded pcap can be played and
      visualized
    
    But ``ouster-cli source <SENSOR HOSTNAME> viz``, or ``ouster-cli source <SENSOR HOSTNAME>
    save`` still can't receive any packets and get the following error::

        ouster.sdk.core.ClientTimeout: No packets received within 1.0s

    Please check your `UFW Firewall`_ settings and try to allow the UDP traffic for ``7502``
    (or whatever the **UDP Port Lidar** is set on the sensor)::

        sudo ufw allow 7502/udp

.. _UFW Firewall: https://help.ubuntu.com/community/UFW


Replay
++++++

Use ``info`` to summarise a captured pcap/osf/bag/mcap file or dataset:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-source-pcap-info]
   :end-before: [doc-etag-cli-source-pcap-info]
   :dedent: 0

For pcap, if the metadata lives in a separate JSON file, pass it with ``--meta`` so the CLI can associate the
correct calibration and configuration with the capture:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-source-pcap-meta]
   :end-before: [doc-etag-cli-source-pcap-meta]
   :dedent: 0

Opening Recordings from Ouster Studio
+++++++++++++++++++++++++++++++++++++

The ``source`` command also accepts links to recordings hosted on `Ouster Studio <https://studio.ouster.com>`_.
When the source is a ``studio.ouster.com`` link, the CLI resolves it to the recording's OSF file and then operates
on it like any local OSF file, so you can chain the usual consumers (``info``, ``viz``, ``save``, ...):

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-source-studio]
   :end-before: [doc-etag-cli-source-studio]
   :dedent: 0

Two kinds of links are supported:

- **Share links** (those containing ``/share/``) are public and require no authentication.
- **Drive links** require you to be signed in to your Ouster organization. The first time you open one, the CLI
  opens a browser window to complete authentication; the resulting credentials are cached under
  ``~/.config/ouster/`` so subsequent commands do not prompt again.

Only recordings that expose an OSF output file can be opened this way — the CLI selects the OSF output and ignores
other available formats.


Single Sensor Options
+++++++++++++++++++++

If required, one can restrict processing to a single sensor index. This is the CLI equivalent of the
``sensor_idx`` parameter (and the underlying ``single()`` method) in the API; see
:ref:`open-source-parameters` for how single-sensor selection works.

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-source-single-sensor]
   :end-before: [doc-etag-cli-source-single-sensor]
   :dedent: 0


Multi Sensor Options
++++++++++++++++++++

Provide a comma-separated list of sensor hostnames to operate on multiple live sensors at once:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-source-multi-host]
   :end-before: [doc-etag-cli-source-multi-host]
   :dedent: 0

When replaying files, combine ``--glob`` with a pattern to iterate through compatible captures:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-source-glob]
   :end-before: [doc-etag-cli-source-glob]
   :dedent: 0


Field Selection
+++++++++++++++

The ``--fields`` option mirrors the SDK APIs that request specific channel fields. Combine it with
``--filter`` to drop incomplete frames while computing ``stats`` or piping data to other consumers:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-source-fields]
   :end-before: [doc-etag-cli-source-fields]
   :dedent: 0

.. _virtual environment: https://docs.python.org/3/library/venv.html
