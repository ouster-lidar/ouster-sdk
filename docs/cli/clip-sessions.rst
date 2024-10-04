Clip Point clouds in the Ouster-CLI
=================================

.. _ouster-cli-clip:


Clip Command
------------

The clip command clips the lidar scan by the given range and streams down the modified lidar
scan to the subsequent commands (like slam, viz, save, etc.). The position of the clip command
in the ouster-cli command chain makes a difference as it only affects the commands that follow it.

To explore the parameters you can use with the clip command, you can use the --help flag:

.. code:: bash

        ouster-cli source <SENSOR_HOSTNAME> / <FILENAME> clip --help


Example Usage
-------------

To keep the points within the 20 m to 50 m range and save the modified lidar scan into a PCAP file
with the visualizer on, run the following command:

.. code:: bash

        ouster-cli source <SENSOR_HOSTNAME> / <FILENAME> clip --min-range 20 --max-range 50 viz save clipped.pcap


Remember, the clip command only affects the commands after it. In the following example, the
viz command runs before the clip command, which means the point cloud modification won't be reflected
in the visualizer but will affect the subsequent save command and the saved PCAP file:

.. code:: bash

        ouster-cli source <SENSOR_HOSTNAME> / <FILENAME> viz clip --min-range 20 --max-range 50 save clipped_2.pcap

In addition to the ``min-range`` and ``max-range`` parameters, the ``clip`` command also includes the
``percent-range`` parameter. This parameter discards points with ranges greater than a specified percentile
in each lidar scan, helping to filter out noise points.

.. code:: bash

        ouster-cli source <SENSOR_HOSTNAME> / <FILENAME> clip --percent-range 99 viz

Combined with SLAM Command
--------------------------

The ``slam`` command also has ``min-range`` and ``max-range`` parameters. When the clip command is
used after the ``slam`` command, the ``clip`` command will, by default, use the range settings specified
in the slam command. However, you can explicitly pass in the range settings to the ``clip`` command to 
apply different ranges to the clip operation.

Note that the range settings in the ``slam`` command only affect the point cloud within the SLAM algorithm.
The slam range settings will not modify the lidar scan and will not affect the other following commands.


Example Usage
-------------

Experiment with the following commands using a pre-recorded PCAP or OSF file:

.. code:: bash

        ouster-cli source <FILENAME> slam clip --min-range 20 --max-range 50 viz save clipped_3.ply
        ouster-cli source <FILENAME> slam --min-range 10 --max-range 100 clip --min-range 20 --max-range 50 viz save clipped_4.ply

You can view the output PLY files using the open source software `CloudCompare`_
For more details about the slam command, refer to the :ref:`SLAM Command <ouster-cli-mapping>`


.. _CloudCompare: https://www.cloudcompare.org/
