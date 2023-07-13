Start mapping with the ouster-cli
=================================


Installation
------------
The Ouster CLI mapping functionality is a part of the Ouster SDK Python
package.

To install the Ouster SDK with the necessary dependencies for running the mapping capabilities:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 install ouster-sdk[mapping]

    .. code-tab:: powershell Windows x64

        PS > py -3 install ouster-sdk[mapping]


Mapping Tools
-------------

After installing the Ouster SDK and mapping dependencies, you can explore various mapping tools
using a connected Ouster sensor or a recorded PCAP file.

Use ``ouster-cli`` to view the available commands and options you can use ``--help``. Try the
following command:

.. code:: bash

        ouster-cli mapping --help

Currently, there are two main commands: ``slam`` and ``convert``. You can further explore each
command by accessing their respective submenus using the ``--help`` flag. For example:

.. code:: bash

        ouster-cli mapping slam --help

SLAM
----
Simultaneous localization and mapping (SLAM) is a technique that enables a system to construct
a map of its surroundings while simultaneously determining its own position on that map.
The Ouster SDK slam command writes lidar scans with per-column poses into an OSF file, an open-source
custom file format for which Ouster provides conversion capabilities, allowing for the
reconstruction of a detailed and precise map later.

Connect to a sensor or use a `Sample PCAP`_

.. note::

        Connecting to an Ouster sensor is covered in the `Networking Guide`_ section of the Ouster
        Sensor Documentation.

Then execute the following command:

.. code:: bash

        ouster-cli mapping slam HOSTNAME / PCAP_FILENAME -o sample.osf viz

.. note::

        Please replace <HOSTNAME> with the corresponding hostname or IP of your sensor, and replace
        <PCAP_FILENAME> with the actual file path and name of the pcap file. Similarly, make the
        necessary substitutions in the subsequent commands.

The terminal will display details such as the output filename and the processing duration. The
output filename must have the .osf extension in order to be used by the convert command.

You can modify settings like point size, color, switching 2D images, or pause playing in the
visualizer. More details can be found at the `Ouster Visualizer`_.


Convert
-------
The convert command converts the SLAM-generated OSF file to a point cloud data file
format such as LAS (.las), PLY (.ply), or PCD (.pcd). The output file format depends on the
extension of the output filename. Let's use the OSF file generated from the SLAM command and convert
it to a PLY file:

.. code:: bash

        ouster-cli mapping convert sample.osf output.ply

The convert command automatically splits and downsamples the trajectory-adjusted point cloud into
several files to prevent exporting a huge size file. The terminal will display details, and you will
see the following printout for each output file:

.. code:: bash

        Output file: output1.ply
        3932160 points accumulated during this period,
        154228 near points are removed [3.92 %],
        1475955 down sampling points are removed [37.54 %],
        2213506 zero range points are removed [56.29 %],
        88471 points are saved [2.25 %].

You can adjust the minimal range, select different fields as values, and change the voxel size by
referring to the ``--help`` flag for more information.

You can use an open source software `CloudCompare`_ to import and view the generated point cloud
data files.


Mapping tools under specific sources
------------------------------------

The same functionality described above in ``slam`` and ``convert`` are available from the source
menu in ``ouster-cli source``.

To run the slam command under the (Sensor and PCAP) sources, use the following command:

.. code:: bash

        ouster-cli source HOSTNAME / PCAP_FILENAME slam -o sample.osf viz

To run the convert command under the (OSF) source and save it as a LAS file, use the following
command:

.. code:: bash

        ouster-cli source sample.osf convert output.las


.. _Sample Pcap: https://static.ouster.dev/sensor-docs/#sample-data

.. _Ouster Visualizer: https://static.ouster.dev/sdk-docs/python/viz/viz-run.html

.. _Networking Guide: https://static.ouster.dev/sensor-docs/image_route1/image_route3/networking_guide/networking_guide.html

.. _CloudCompare: https://www.cloudcompare.org/
