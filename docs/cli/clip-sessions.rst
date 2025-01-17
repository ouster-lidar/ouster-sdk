Clip Point clouds values in Ouster-CLI
======================================

.. _ouster-cli-clip:


Clip Command
------------

The ``clip`` command can be used to limit the range of values of different scan fields. The outcome of this command
can then be consumed by other downstream commands operation (like slam, viz, save, etc.). The position of the ``clip``
command in the ouster-cli command chain makes a difference as it only affects operations that come afterwords.

To explore the parameters you can use with the clip command, you can use the --help flag:

.. code:: bash

        ouster-cli source <SENSOR_HOSTNAME> / <FILENAME> clip --help


Example Usage
-------------

To keep the points within the 20 m to 50 m range and save the modified lidar scan into a PCAP file
with the visualizer on, run the following command:

.. code:: bash

        ouster-cli source <SENSOR_HOSTNAME> / <FILENAME> clip RANGE,RANGE2 20m:50m viz save clipped.pcap


Remember, the ``clip`` command only affects the commands after it. In the following example, the
viz command runs before the clip command, which means the point cloud modification won't be reflected
in the visualizer but will affect the subsequent save command and the saved PCAP file:

.. code:: bash

        ouster-cli source <SENSOR_HOSTNAME> / <FILENAME> viz clip RANGE,RANGE2 20m:50m save clipped_2.pcap


Combined with SLAM Command
--------------------------

The ``slam`` command  has ``min-range`` and ``max-range`` parameters. When the clip command is used after
the ``slam`` command, the ``clip`` command will, by default, use the range settings specified in the slam
command. However, you can explicitly pass in the range settings to the ``clip`` command to  apply different
ranges to the clip operation.

Note that the range settings in the ``slam`` command only affect the point cloud within the SLAM algorithm.
The slam range settings will not modify the lidar scan and will not affect the other following commands.


Example Usage
-------------

Experiment with the following commands using a pre-recorded PCAP or OSF file:

.. code:: bash

        ouster-cli source <FILENAME> slam clip RANGE,RANGE2 20m:50m viz save clipped_3.ply
        ouster-cli source <FILENAME> slam --min-range 10 --max-range 100 clip RANGE,RANGE2 20m:50m viz save clipped_4.ply

You can view the output PLY files using the open source software `CloudCompare`_
For more details about the slam command, refer to the :ref:`SLAM Command <ouster-cli-mapping>`


.. _CloudCompare: https://www.cloudcompare.org/
