Start mapping with the ouster-cli
=================================


Installation
------------
The Ouster CLI mapping functionality is a part of the Ouster SDK Python
package.

To install the Ouster SDK with the mapping capabilities:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 install ouster-sdk

    .. code-tab:: powershell Windows x64

        PS > py -3 install ouster-sdk


Mapping Tools
-------------

After installing the Ouster SDK and mapping dependencies, you can explore various mapping tools
using a connected Ouster sensor or a PCAP/OSF file.

Use ``ouster-cli`` to view the available commands and options you can use ``--help``. Try the
following command:

.. code:: bash

        ouster-cli source HOSTNAME / FILENAME --help

Currently, there are two main commands: ``slam`` and ``convert``. You can further explore each
command by accessing their respective submenus using the ``--help`` flag. For example:

.. code:: bash

        ouster-cli source HOSTNAME / FILENAME slam --help

SLAM
----
Simultaneous localization and mapping (SLAM) is a technique that enables a system to construct
a map of its surroundings while simultaneously determining its own position on that map.
The Ouster SDK slam command writes lidar scans with per-column poses into an OSF file, an open-source
custom file format for which Ouster provides conversion capabilities, allowing for the
reconstruction of a detailed and precise map later.

Connect to a sensor or use a PCAP/OSF file :ref:`Download Sample PCAP File <sample-data-download>`

.. note::

        Connecting to an Ouster sensor is covered in the `Networking Guide`_ section of the Ouster
        Sensor Documentation.

Then execute the following command:

.. code:: bash

        ouster-cli source HOSTNAME / FILENAME slam viz -o sample.osf

.. note::

        Please replace <HOSTNAME> with the corresponding hostname or IP of your sensor, and replace
        <PCAP_FILENAME> with the actual file path and name of the pcap file. Similarly, make the
        necessary substitutions in the subsequent commands.

The terminal will display details such as the output filename and the processing duration. The
output filename must have the .osf extension in order to be used by the convert command.

You can modify settings like point size, color, switching 2D images, pause playing in the
visualizer and display accumulated scans. More details can be found at the
:ref:`Ouster Visualizer <viz-run>` and :ref:`Scans Accumulator <viz-scans-accum>`


Convert
-------
The convert command converts the SLAM-generated OSF file to a point cloud data file
format such as LAS (.las), PLY (.ply), or PCD (.pcd). The output file format depends on the
extension of the output filename. Let's use the OSF file generated from the SLAM command and convert
it to a PLY file:

.. code:: bash

        ouster-cli source sample.osf convert output.ply

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


.. code:: bash

        ouster-cli source sample.osf convert output.las


.. _Networking Guide: https://static.ouster.dev/sensor-docs/image_route1/image_route3/networking_guide/networking_guide.html

.. _CloudCompare: https://www.cloudcompare.org/
