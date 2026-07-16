Converting PCAPs to Other Formats
==================================

The Ouster Python SDK provides several examples to convert PCAP files to other common point cloud formats such as CSV, LAS, PCD, and PLY.

.. _conversion-pcap:

.. _ex-pcap-to-csv:

PCAPs to CSV
------------

Sometimes we want to get a point cloud (``XYZ`` + other fields) as a ``CSV`` file for further
analysis with other tools.

To convert the first ``5`` frames of our sample data from a pcap file, you can try:

.. tab-set::
   :sync-group: os-commands

   .. tab-item:: Linux/macOS
      :sync: unix

      .. code::

        $ ouster-cli source --meta $SAMPLE_DATA_JSON_PATH $SAMPLE_DATA_PCAP_PATH slice 0:5 save output.csv

   .. tab-item:: Windows x64
      :sync: windows

      .. code::

        PS > ouster-cli.exe source --meta $SAMPLE_DATA_JSON_PATH $SAMPLE_DATA_PCAP_PATH slice 0:5 save output.csv


The following function implements the pcap to csv conversion above.

.. literalinclude:: /../python/src/ouster/cli/plugins/source_save.py
    :start-after: [doc-stag-pcap-to-csv]
    :end-before: [doc-etag-pcap-to-csv]
    :class: doc-snippet
    :caption: View on `GitHub <|github-src|python/src/ouster/cli/plugins/source_save.py>`__
    :dedent:

Because we stored the frame as structured 2D images, we can easily recover it by loading it back into
a ``numpy.ndarray`` and continuing to use it as a 2D image.

.. code:: python

    import numpy as np

    # read array from CSV
    frame = np.loadtxt('output_s0_0.csv', delimiter=',')

    # convert back to "fat" 2D image [H x W x num_fields] shape
    frame = frame.reshape((128, -1, frame.shape[1]))

We used ``128`` while restoring 2D image from a CSV file because it's the number of channels of our
``OS-1-128.pcap`` sample data recording.


.. _ex-pcap-to-las:

PCAPs to LAS
------------

To convert to the first ``5`` frames of our sample data from a pcap file to ``LAS``, you can try:

.. tab-set::
   :sync-group: os-commands

   .. tab-item:: Linux/macOS
      :sync: unix

      .. code::

        $ python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH pcap-to-las --frame-num 5

   .. tab-item:: Windows x64
      :sync: windows

      .. code::

        PS > py -3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH pcap-to-las --frame-num 5

Checkout the :func:`.examples.pcap.pcap_to_las` documentation for the example source code.

PCAPs to PCD
------------

To convert to the first ``5`` frames of our sample data from a pcap file to ``PCD``, you can try:

.. tab-set::
   :sync-group: os-commands

   .. tab-item:: Linux/macOS
      :sync: unix

      .. code::

        $ python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH pcap-to-pcd --frame-num 5

   .. tab-item:: Windows x64
      :sync: windows

      .. code::

        PS > py -3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH pcap-to-pcd --frame-num 5

Checkout the :func:`.examples.pcap.pcap_to_pcd` documentation for the example source code.

PCAPs to PLY
------------

Here we will reuse the PCAP to PCD function that uses Open3d and will exploit the extensive `Open3d
File IO`_ that gives us an easy way to save the loaded point cloud to ``PLY``. Alternative ways are available via `plyfile`_ library.

To convert to the first ``5`` frames of our sample data from a pcap file to ``PLY``, you can try:

.. tab-set::
   :sync-group: os-commands

   .. tab-item:: Linux/macOS
      :sync: unix

      .. code::

        $ python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH pcap-to-ply --frame-num 5

   .. tab-item:: Windows x64
      :sync: windows

      .. code::

        PS > py -3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH pcap-to-ply --frame-num 5

Checkout the :func:`.examples.pcap.pcap_to_ply` documentation for the example source code.

.. _Open3d File IO: http://www.open3d.org/docs/release/tutorial/geometry/file_io.html#Point-cloud
.. _plyfile: https://pypi.org/project/plyfile/
