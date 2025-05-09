.. _common commands:


Common Use Cases
----------------

One of the goals of ``ouster-cli`` is to easily allow the most common sensor and recorded data
interactions. We cover some common use cases here, listed alphabetically. Please note that wherever
<SENSOR_HOSTNAME> is used, you are expected to substitute in your sensor's hostname or IP, e.g.,
``os1-991913000010.local``.


Benchmarking
++++++++++++

To help users obtain performance metrics for the Ouster SDK, we
provide a benchmarking utility which will download `8 seconds of OS2 data`_, gather system info,
time various data operations, and generate a report which you can share with others. Give it a try!

.. code:: bash
    
    $ ouster-cli util benchmark

This will generate a report in a directory named ouster-bench, which will be located in the current working directory.

.. _8 seconds of OS2 data: https://data.ouster.dev/drive/7377


Discovering sensors on local network
++++++++++++++++++++++++++++++++++++

Sensors announce their presence on the network using Multicast Domain Name Service (mDNS). Use
helper utility command ``discover`` to list names and IPs of all available sensors on the local
network:

.. code:: bash

    $ ouster-cli discover

Collecting Metadata
+++++++++++++++++++

Sensor metadata, necessary for interpreting and parsing the pcap data, can be collected from sensors
using:

.. code:: bash

    $ ouster-cli source <SENSOR_HOSTNAME> metadata > <SENSOR_HOSTNAME>.json

This will generate a ``.json`` file named ``<SENSOR_HOSTNAME>.json`` with the metadata inside. To
output it to a differently named file, simply change ``<SENSOR_HOSNTAME>.json`` to
``<FILL_IN_ALTERNATE_NAME>.json``. You can also print the metadata to screen by removing ``>`` and
everything after it in the command.


Configuring Your Sensor
+++++++++++++++++++++++

``ouster-cli`` provides utilities for configuring your sensor with configuration parameters such as
``lidar_mode`` and ``azimuth_window``.

To quickly auto-configure a sensor with with standard ports, azimuth window, operating mode, and
auto udp dest:

.. code:: bash

    $ ouster-cli source <SENSOR_HOSTNAME> config

But what if you want to specify the ports and lidar_mode? You can use the ``sensor config`` command thusly:

.. code:: bash

    $ ouster-cli source <SENSOR_HOSTNAME> config lidar_mode 1024x10 udp_port_lidar 29847

.. note::

    Multiple ``<PARAM> <VALUE>`` pairs can be passed this way!

You may have a configuration that you want to use repeatedly. Typing these in at the command line
every time would be annoying. You can instead save your config to a json, named CONFIG_JSON, here,
and run:

.. code:: bash

    $ ouster-cli source <SENSOR_HOSTNAME> config -c <CONFIG JSON>

And finally, you may wish to save a configuration after setting your sensor up perfectly. To do so:

.. code:: bash

    $ ouster-cli source <SENSOR_HOSTNAME> config -d

That will print your json to stdout. Use ``>`` to redirect it to a file!


Recording Pcaps
+++++++++++++++

To record data from a udp port (7502 by default) to a pcap file in the current directory and write
the metadata to a json file with the same name, simply use:

.. code:: bash

    $ ouster-cli source <SENSOR_HOSTNAME> save_raw .pcap

This will record until you keyboard interrupt, i.e., use ``CTRL+C``. You can also set it to record
a specific length or number of packets, or to use different ports for lidar and IMU data. As always
with ``ouster-cli``, use ``--help`` to discover how those options work.


Visualizing Lidar Data
++++++++++++++++++++++

The following visualizes lidar data arriving on a udp port. Note that you may have to use
``ouster-cli source <SENSOR_HOSTNAME> config`` first to configure your sensor properly.

.. code:: bash

    $ ouster-cli source <SENSOR_HOSTNAME> viz


The following replays lidar data saved in a pcap file and visualizes the output. It will looks for a
metadata json file with the same name as PCAP FILE by default, but you can specify a file using ``-m
<METADATA JSON>``.

.. code:: bash

    $ ouster-cli source <PCAP FILE> viz


Masking Lidar Data
++++++++++++++++++

The following command applies an image mask ``MASK-IMAGE`` to the ``RANGE`` data field of incoming
scans:

.. code:: bash

    $ ouster-cli source <SOURCE_URL> mask --fields RANGE <MASK-IMAGE> viz


The ``MASK-IMAGE`` is expected to be of composed solely of black and white pixels; black pixels
represent the pixels that will be zeroed out and white pixels represent areas of the RANGE that
will stay intact. The ``MASK-IMAGE`` is expected to have the same size of the streamed LidarScans.
If not the command will scale the mask image to the same size as the incoming LidarScans.
The ``MASK-IMAGE`` will be applied to all sensors in case of a multi-sensor dataset.


Reduce Beam Count
+++++++++++++++++
Use the ``reduce`` command to reduce the lower vertical resolution or beam count of stream LidarScans.
For example, let's assume you have an OS-1-128 Ouster sensor which has 128 beams, using the following
command you can reduce the effective vertical resolution of the sensor to 32:

.. code:: bash

    $ ouster-cli source <SOURCE_URL> reduce --fields 32 viz


The reduced LidarScans will applied to the rest of the chain, that means if you chain a ``save`` command
afterwords the generated file will have LidarScans with 32 beams only.
One thing to note is that the beams are sampled uniformally across the original beam count.

.. note::

    Currently, the ``reduce`` command can't occure more than once in the ouster-cli command chain and needs
    to be the very first command after the ``source`` args.
