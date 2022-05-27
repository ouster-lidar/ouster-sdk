=============================
Download Recorded Sample Data
=============================

If you haven't downloaded the ``ouster-sdk``, please follow the :ref:`Python Installation
<installation-python>` instructions.

Download Data
=============

.. _sample-data-download:

..
   [start-download-instructions]

Download one of the following samples of recorded Ouster data and unzip the contents:

.. _dual-returns-snippets:

   * `OS0 128 Rev 06 Urban Drive (Dual Returns)`_ [151 MB] (`preview OS0 <https://data.ouster.dev/share/QBBY706GG0R6ZOG1?utm_source=sdk&utm_medium=sdk>`_)
   * `OS1 128 Rev 06 Urban Drive (Dual Returns)`_ [173 MB] (`preview OS1 <https://data.ouster.dev/share/D03HJ28ZX3245FQQ?utm_source=sdk&utm_medium=sdk>`_)
   * `OS2 128 Rev 06 Urban Drive (Dual Returns)`_ [190 MB] (`preview OS2 <https://data.ouster.dev/share/NNRMR0PCGVEMQKYM?utm_source=sdk&utm_medium=sdk>`_)
   * `OS1 128 Rev 06 Urban Drive (Low Bandwidth)`_ [95 MB] (`preview OS1 LB <https://data.ouster.dev/share/SORYN8B6OAF0BVDL?utm_source=sdk?utm_medium=sdk&frame=53>`_)
   * `OS2 128 Rev 05 Bridge`_ [82 MB] (`preview bridge <https://data.ouster.dev/share/U7W1P8MFUEOKT61G?utm_source=sdk&utm_medium=sdk>`_)


.. _OS0 128 Rev 06 Urban Drive (Dual Returns): https://data.ouster.io/sdk-samples/Rev-06-fw23/OS0-128_Rev-06_fw23_Urban-Drive_Dual-Returns.zip
.. _OS1 128 Rev 06 Urban Drive (Dual Returns): https://data.ouster.io/sdk-samples/Rev-06-fw23/OS1-128_Rev-06_fw23_Urban-Drive_Dual-Returns.zip
.. _OS2 128 Rev 06 Urban Drive (Dual Returns): https://data.ouster.io/sdk-samples/Rev-06-fw23/OS2-128_Rev-06_fw23_Urban-Drive_Dual-Returns.zip
.. _OS1 128 Rev 06 Urban Drive (Low Bandwidth): https://data.ouster.io/sdk-examples/Rev-06-fs23/OS1-128_Rev-06-fw23_Urban-Drive_Low-Bandwidth.zip
.. _OS2 128 Rev 05 Bridge: https://data.ouster.io/sdk-samples/Rev-05/OS2-128_Rev-05_Bridge/OS2-128_Rev-05_Bridge.zip 

In your unzipped directory, you should have two files, one ``.pcap`` file and one ``.json`` file.
For example, in the unzipped recorded sample of OS1-128 Rev 06 data you should find:

  * ``OS1-128_Rev-06_fw23_Urban-Drive_Dual-Returns.pcap``
  * ``OS1-128_Rev-06_fw23_Urban-Drive_Dual-Returns.json``

We will use ``SAMPLE_DATA_PCAP_PATH`` to refer to this pcap and ``SAMPLE_DATA_JSON_PATH`` to this
json in the following.  You may find it convenient to assign the paths appropriately in your
console.

..
   [end-download-instructions]

Visualize It!
=============

If you've installed the ``ouster-sdk`` (see :ref:`Python Installation <installation-python>`) then
you're all set to visualize with::
        
   $ simple-viz --pcap $SAMPLE_DATA_PCAP_PATH --meta $SAMPLE_DATA_JSON_PATH
       
You should get a view similar to:

.. figure:: /images/simple-viz.png
    :align: center

    Ouster ``simple-viz`` visualization of OS1 128 Rev 6 sample data

You can control your visualizer with mouse and keyboard:

.. include:: /python/viz/viz-run.rst
    :start-after: [start-simple-viz-keymap]
    :end-before: [end-simple-viz-keymap]

Congratulations! You've installed and visualized with the Ouster Python SDK!

For more on ``simple-viz``, please see :doc:`python/viz/viz-run`
