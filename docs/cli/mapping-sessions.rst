Start mapping with the Ouster-CLI
=================================


.. _ouster-cli-mapping:

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

        ouster-cli source <SENSOR_HOSTNAME> / <FILENAME> --help

Currently, there are two main commands: ``slam`` and ``save``. You can further explore each
command by accessing their respective submenus using the ``--help`` flag. For example:

.. code:: bash

        ouster-cli source <SENSOR_HOSTNAME> / <FILENAME> slam --help


SLAM Command
------------

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

        ouster-cli source <SENSOR_HOSTNAME> / <FILENAME> slam viz -o sample.osf

.. note::

        Please replace <HOSTNAME> with the corresponding hostname or IP of your sensor, and replace
        <FILENAME> with the actual file path and name of the PCAP/OSF file. Similarly, make the
        necessary substitutions in the subsequent commands.

The terminal will display details such as the output filename and the processing duration. The
output filename must have the .osf extension in order to be used by the ``save`` command.

You can adjust settings such as point size, color, switch between 2D images, and pause playback in the visualizer, among other options. More details can be found at the :ref:`Ouster Visualizer <viz-run>`


Accumulated Scan in SLAM command visulizer
------------------------------------------

Within the Ouster Visualizer, there is a visualization feature known as **ScansAccumulator**. This functionality represents a continuation of efforts to visualize lidar data by incorporating SLAM-generated poses stored within the ``LidarScan.pose`` property."

Available view modes
~~~~~~~~~~~~~~~~~~~~~

There are three view modes of **ScansAccumulator**, that may be enabled/disabled depending on
it's params and the data that is passed throught it:

   * **poses** (or **TRACK**), key ``8`` - all scan poses in a trajectory/path view (available only
     if poses data is present in scans)
   * **scan map** (or **MAP**), key ``7`` - overall map view with select ratio of random points
     from every scan (available for scans with/without poses)
   * **scan accum** (or **ACCUM**), key ``6`` - accumulated *N* scans (key frames) that is picked
     according to params (available for scans with/without poses)


Key bindings
~~~~~~~~~~~~~

Keyboard controls available with **ScansAccumulator**:

    ==============  =============================================================
        Key         What it does
    ==============  =============================================================
    ``6``           Toggle scans accumulation view mode (ACCUM)
    ``7``           Toggle overall map view mode (MAP)
    ``8``           Toggle poses/trajectory view mode (TRACK)
    ``k / K``       Cycle point cloud coloring mode of accumulated clouds or map
    ``g / G``       Cycle point cloud color palette of accumulated clouds or map
    ``j / J``       Increase/decrease point size of accumulated clouds or map
    ==============  =============================================================

Ouster CLI **ScansAccumulator** options:

  * ``--accum-num N`` - accumulate *N* scans (default: ``0``)
  * ``--accum-every K`` - accumulate every *Kth* scan (default: ``1``)
  * ``--accum-every-m M`` - accumulate a scan every *Mth* meters traveled (default: ``None``)
  * ``--accum-map`` - enable the overall map accumulation, select some percentage of points from
    every scan (default: disabled)
  * ``--accum-map-ratio R`` - set *R* as a ratio of points to randomly select from every scan
    (default: ``0.001`` (*0.1%*))

Dense accumulated clouds view (with every point of a scan)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To obtain the densest view use the ``--accum-num N --accum-every 1`` params where ``N`` is the
number of clouds to accumulate (``N`` up to 100 is generally small enough to avoid slowing down the viz interface)::

   ouster-cli source <SENSOR_HOSTNAME> / <FILENAME> slam viz --accum-num 20 -o sample.osf

and the dense accumulated clouds result:

.. figure:: /images/scans_accum_dense_every.png

   Dense view of 20 accumulated scans during the ``slam viz`` run


Overall map view (with poses)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

One of the main tasks we frequently need is a preview of the overall map. We can test this by using the generated OSF file, which was created with the above command and contains the SLAM-generated ``LidarScan.pose`` property.
::

   ouster-cli source sample.osf viz --accum-num 20 \
   --accum-every 0 --accum-every-m 10.5 --accum-map -r 0 -e stop


Here is a preview example of the overall map generated from the accumulated scan results. By utilizing the '-e stop' option, the visualizer stops once the replay process finishes, displaying the preview of the lidar trajectory:

.. figure:: /images/scans_accum_map_all_scan.png

   Data fully replayed with map and accum enabled (last current scan is displayed here in grey
   palette)


Save Command
------------

The ``save`` command can be used to convert the SLAM-generated OSF file to a point cloud data file
format such as LAS (.las), PLY (.ply), or PCD (.pcd). The output file format depends on the
extension of the output filename. Let's use the OSF file generated from the SLAM command
and convert it to a PLY file:

.. code:: bash

        ouster-cli source sample.osf save output.ply

The ``save`` command automatically splits and downsamples the trajectory-adjusted point cloud into
several files to prevent exporting a huge size file. The terminal will display details, and you
will see the following printout for each output file:

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


.. _Networking Guide: https://static.ouster.dev/sensor-docs/image_route1/image_route3/networking_guide/networking_guide.html

.. _CloudCompare: https://www.cloudcompare.org/
