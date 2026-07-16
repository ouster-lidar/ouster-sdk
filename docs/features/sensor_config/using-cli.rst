Using the CLI
=============
.. _common commands:


One of the goals of ``ouster-cli`` is to easily allow the most common sensor and recorded data
interactions. We cover some common use cases here, listed alphabetically. Please note that wherever
<SENSOR_HOSTNAME> is used, you are expected to substitute in your sensor's hostname or IP, e.g.,
``os1-991913000010.local``.

Discovering sensors on local network
++++++++++++++++++++++++++++++++++++

Sensors announce their presence on the network using Multicast Domain Name Service (mDNS). Use
helper utility command ``discover`` to list names and IPs of all available sensors on the local
network:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-discover]
   :end-before: [doc-etag-cli-discover]
   :dedent: 0

    
Configuring Your Sensor
+++++++++++++++++++++++

``ouster-cli`` provides utilities for configuring your sensor with configuration parameters such as
``lidar_mode`` and ``azimuth_window``.

To quickly auto-configure a sensor with with standard ports, azimuth window, operating mode, and
auto udp dest:


.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-auto-config]
   :end-before: [doc-etag-cli-auto-config]
   :dedent: 0

But what if you want to specify the ports and lidar_mode? You can use the ``config`` command as shown below:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-custom-config]
   :end-before: [doc-etag-cli-custom-config]
   :dedent: 0

.. note::

    Multiple ``<PARAM> <VALUE>`` pairs can be passed this way!


.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-custom-config-2]
   :end-before: [doc-etag-cli-custom-config-2]
   :dedent: 0

Save config to a file
+++++++++++++++++++++

You may have a configuration that you want to use repeatedly. Typing these in at the command line
every time would be annoying. You can instead save your config to a json, named ``CONFIG_JSON``, by running:


.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-config-file]
   :end-before: [doc-etag-cli-config-file]
   :dedent: 0

And finally, you may wish to save a configuration after setting your sensor up perfectly. To do so:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-get-config]
   :end-before: [doc-etag-cli-get-config]
   :dedent: 0

That will print your json to stdout. Use ``>`` to redirect it to a file!
