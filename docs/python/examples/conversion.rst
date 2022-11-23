==================================
Converting PCAPs to Other Formats
==================================

.. contents::
   :local:
   :depth: 2

.. _conversion-pcap:

.. _ex-pcap-to-csv:

PCAPs to CSV
=============

Sometimes we want to get a point cloud (``XYZ`` + other fields) as a ``CSV`` file for further
analysis with other tools.

To convert the first ``5`` scans of our sample data from a pcap file, you can try:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH pcap-to-csv --scan-num 5

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH pcap-to-csv --scan-num 5


The source code of an example below:

.. literalinclude:: /../python/src/ouster/sdk/examples/pcap.py
    :start-after: [doc-stag-pcap-to-csv]
    :end-before: [doc-etag-pcap-to-csv]
    :emphasize-lines: 37-41
    :linenos:
    :dedent:

Because we stored the scan as structured 2D images, we can easily recover it by loading it back into
a ``numpy.ndarray`` and continuing to use it as a 2D image.

.. code:: python

    import numpy as np

    # read array from CSV
    frame = np.loadtxt('my_frame_00000.csv', delimiter=',')

    # convert back to "fat" 2D image [H x W x num_fields] shape
    frame = frame.reshape((128, -1, frame.shape[1]))

We used ``128`` while restoring 2D image from a CSV file because it's the number of channels of our
``OS-1-128.pcap`` sample data recording.


.. _ex-pcap-to-las:

PCAPs to LAS
=============

To convert to the first ``5`` scans of our sample data from a pcap file to ``LAS``, you can try:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH pcap-to-las --scan-num 5

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH pcap-to-las --scan-num 5

Checkout the :func:`.examples.pcap.pcap_to_las` documentation for the example source code.

PCAPs to PCD
=============

To convert to the first ``5`` scans of our sample data from a pcap file to ``PCD``, you can try:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH pcap-to-pcd --scan-num 5

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH pcap-to-pcd --scan-num 5

Checkout the :func:`.examples.pcap.pcap_to_pcd` documentation for the example source code.

PCAPs to PLY
=============

Here we will reuse the PCAP to PCD function that uses Open3d and will exploit the extensive `Open3d
File IO`_ that gives us an easy way to save the loaded point cloud to ``PLY``. Alternative ways are available via `plyfile`_ library.

To convert to the first ``5`` scans of our sample data from a pcap file to ``PLY``, you can try:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH pcap-to-ply --scan-num 5

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH pcap-to-ply --scan-num 5

Checkout the :func:`.examples.pcap.pcap_to_ply` documentation for the example source code.

.. _Open3d File IO: http://www.open3d.org/docs/release/tutorial/geometry/file_io.html#Point-cloud
.. _plyfile: https://pypi.org/project/plyfile/