Using the CLI
=============

.. _ouster-cli-clip:

Rewriting an OSF Subset
-----------------------

You can resave a subset of an OSF file—for example the first N frames—by chaining ``slice`` and
``save``. The CLI currently writes all available fields; reducing the field set (e.g., RANGE only)
requires the Python/C++ APIs.


.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-slice-osf]
   :end-before: [doc-etag-cli-slice-osf]
   :dedent: 0
   :caption: Copy a portion of ``{SAMPLE_DATA_OSF_PATH}`` to a new OSF file


Clip Command
------------

The ``clip`` command can be used to limit the range of values of different frame fields. The outcome of this command
can then be consumed by other downstream commands operation (like slam, viz, save, etc.). The position of the ``clip``
command in the ouster-cli command chain makes a difference as it only affects operations that come afterwards.

To explore the parameters you can use with the clip command, you can use the --help flag:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-clip-help]
   :end-before: [doc-etag-clip-help]
   :dedent: 0
  
.. rubric:: **Example Usage**

To keep the points within the 20 m to 50 m range and save the modified lidar frame into a PCAP file
with the visualizer on, run the following command:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-clip]
   :end-before: [doc-etag-cli-clip]
   :dedent: 0
  

Remember, the ``clip`` command only affects the commands after it. In the following example, the
viz command runs before the clip command, which means the point cloud modification won't be reflected
in the visualizer but will affect the subsequent save command and the saved PCAP file:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-clip-viz]
   :end-before: [doc-etag-clip-viz]
   :dedent: 0
  

Combined with SLAM Command
--------------------------

The ``slam`` command  has ``min-range`` and ``max-range`` parameters. When the clip command is used after
the ``slam`` command, the ``clip`` command will, by default, use the range settings specified in the slam
command. However, you can explicitly pass in the range settings to the ``clip`` command to  apply different
ranges to the clip operation.

Note that the range settings in the ``slam`` command only affect the point cloud within the SLAM algorithm.
The slam range settings will not modify the lidar frame and will not affect the other following commands.


.. rubric:: **Example Usage**

Experiment with the following commands using a pre-recorded PCAP or OSF file:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-clip-slam-viz]
   :end-before: [doc-etag-clip-slam-viz]
   :dedent: 0
  

You can view the output PLY files using the open source software `CloudCompare`_
For more details about the slam command, refer to the :ref:`SLAM Command <ouster-cli-mapping>`


.. _CloudCompare: https://www.cloudcompare.org/


Filter Point clouds values
--------------------------

.. _ouster-cli-filter:


The ``filter`` command offers a rich set of filter options that can be used to suppress or replace the measurements
values of selected fields based on a certain predicate. Pixels values that match the predicate will be replaced by
the value of the option ``--invalid-value`` (default is zero) .

The current syntax:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-filter]
   :end-before: [doc-etag-filter]

Where:

- ``SOURCE`` can be a sensor hostname or a PCAP file.
- ``AXIS_FIELD`` can be either a field name such as ``RANGE``, ``REFLECTIVITY`` or a axis in the cartesian coordinates
  ``{X, Y, Z}`` or image coordinates ``{U, V}``.
- ``INDICES`` The indices specifiy a range of values (e.g., ``0:10``) that map to the values of the ``AXIS_FIELD``. Any
  value of the choosen ``AXIS_FIELD`` that matches the ``INDICES`` will be replaced by zero unless the option ``--invalid-value``
  is set to a different value.
- ``[OPTIONS]`` current options include:

  - ``--invalid-value``: The value to replace the pixels that match the predicate. Default is zero.
  - ``--filtered-fields``: A comma-separated list of fields to apply the filter to. If not specified, the filter will
    be applied to all fields of the frame.


.. rubric:: **Example Usage**

1) Filter based on field values:

Consider the image from one of Ouster example sequences:

.. figure:: /images/filter-example-sequence.png
   :alt: Default point cloud scene
   :align: center
   :width: 100%

   Default point cloud scene

If we apply a filter to the same sequence using the ``REFLECTIVITY`` channel and a range of value from 0 to 50 as the filter
command predicate, we can highlight points in the low cloud with higher reflectivity in the scene:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-filter-rfl]
   :end-before: [doc-etag-filter-rfl]
   :dedent: 0
   
.. figure:: /images/filter-example-reflectivity.png
   :alt: Filter by Reflectivity
   :align: center
   :width: 100%

   Filter by Reflectivity

  
1) Filter based on cartesian coordinates:

Another way to filter the point cloud is to use the cartesian coordinates of the points. For example, imagine we want to
only view the points that are +/-1 meter up or down from the sensor. We can use the ``Z`` axis to filter these points as
follows:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-filter-z]
   :end-before: [doc-etag-filter-z]
   :dedent: 0

After applying this filter the resulting point cloud will look like this:

.. figure:: /images/filter-example-z.png
   :alt: Filter by Z
   :align: center
   :width: 100%

   Filter by Z

As you can see, in this example we cascaded the filter command over the ``Z`` axis to suppress the points
of the pointcloud that are outside the range of [-1m, 1m].

3) Filter based on image coordinates:

One more way to use the filter command is through image coordinates of the input LidarFrame. consider the following example:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-filter-coord]
   :end-before: [doc-etag-filter-coord]
   :dedent: 0

This command will filter the point cloud to only include points that are in the U coordinate range of 512 to 1536. The
resulting point cloud will look like this:

.. figure:: /images/filter-example-v.png
   :alt: Filter by V
   :align: center
   :width: 100%

   Filter by V

This can be useful to mask out certain columns (``V``) or certain beams (``U``) of the LidarFrame. 

.. note::

        When using the image coordinates for filtering PointClouds, it is important to know the dimensions of the LidarFrame as
        these coordinates are absolute values in the image space and they don't wrap around. For example, if the LidarFrame has
        a size of 128x1024, then the valid range for ``U`` is [0, 128] and for ``V`` is [0, 1024]. Using values beyond that will
        fail.


Here is one final example that shows the use of the options ``--filtered-fields`` and ``--invalid-value``

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-filter-opts]
   :end-before: [doc-etag-filter-opts]
   :dedent: 0

The ``--filtered-fields`` option allows you to limit the channels that this filter will be applied to, while the ``--invalid-value``
option can be used to choose a value other than zero when overriding pixel values of selected channels. Following is the outcome of
the command:

.. figure:: /images/filter-example-filtered-fields-invalid.png
   :alt: Filter by fields
   :align: center
   :width: 100%

   Filter by fields


Masking Lidar Data
------------------

The following command applies an image mask ``MASK-IMAGE`` to the ``RANGE`` data field of incoming
frames:


.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-mask]
   :end-before: [doc-etag-mask]
   :dedent: 0

The ``MASK-IMAGE`` is expected to be of composed solely of black and white pixels; black pixels
represent the pixels that will be zeroed out and white pixels represent areas of the RANGE that
will stay intact. The ``MASK-IMAGE`` is expected to have the same size of the streamed LidarFrames.
If not the command will scale the mask image to the same size as the incoming LidarFrames.
The ``MASK-IMAGE`` will be applied to all sensors in case of a multi-sensor dataset.


Reduce Beam Count
-----------------

Use the ``reduce`` command to reduce the lower vertical resolution or beam count of stream LidarFrames.
For example, let's assume you have an OS-1-128 Ouster sensor which has 128 beams, using the following
command you can reduce the effective vertical resolution of the sensor to 32:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-reduce]
   :end-before: [doc-etag-reduce]
   :dedent: 0

The reduced LidarFrames will applied to the rest of the chain, that means if you chain a ``save`` command
afterwards the generated file will have LidarFrames with 32 beams only.
One thing to note is that the beams are sampled uniformally across the original beam count.

.. note::

    Currently, the ``reduce`` command can't occur more than once in the ouster-cli command chain and needs
    to be the very first command after the ``source`` args.



