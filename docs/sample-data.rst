==============================
Download Recorded Sample Data
==============================

Download Data
==============

.. todo::
   Update with 2.3 and add links to data app.

.. _sample-data-download:

..
   [start-download-instructions]

Download one of the following samples of recorded Ouster data and unzip the contents:

.. _dual-returns-snippets:

   * `OS0 128 Rev 06 Urban Drive (Dual Returns)`_ [151 MB] (`preview TBD <https://data.ouster.dev/share/WPPFCY9T0EY7UHHO?utm_source=sdk&utm_medium=sdk>`_)
   * `OS1 128 Rev 06 Urban Drive (Dual Returns)`_ [173 MB] (`preview TBD <https://data.ouster.dev/share/JOCA49LZ382DX71D?utm_source=sdk&utm_medium=sdk>`_)
   * `OS2 128 Rev 06 Urban Drive (Dual Returns)`_ [190 MB] (`preview TBD <https://data.ouster.dev/share/D42MQAJ6KZ8ID0ON?utm_source=sdk&utm_medium=sdk>`_)
   * `OS2 128 Rev 05 Bridge`_ [82 MB]

.. _OS0 128 Rev 06 Urban Drive (Dual Returns): https://data.ouster.io/sdk-samples/Rev-06-fw23/OS0-128_Rev-06_fw23_Urban-Drive_Dual-Returns.zip
.. _OS1 128 Rev 06 Urban Drive (Dual Returns): https://data.ouster.io/sdk-samples/Rev-06-fw23/OS1-128_Rev-06_fw23_Urban-Drive_Dual-Returns.zip
.. _OS2 128 Rev 06 Urban Drive (Dual Returns): https://data.ouster.io/sdk-samples/Rev-06-fw23/OS2-128_Rev-06_fw23_Urban-Drive_Dual-Returns.zip
.. _OS2 128 Rev 05 Bridge: https://data.ouster.io/sdk-samples/Rev-05/OS2-128_Rev-05_Bridge/OS2-128_Rev-05_Bridge.zip 

In your unzipped directory, you should have two files, one ``.pcap`` file and one ``.json`` file.
For example, in the unzipped recorded sample of OS1-128 Rev 06 data you should find:

  * ``OS1-128_Rev-06_fw23_Urban-Drive_Dual-Returns.pcap``
  * ``OS1-128_Rev-06_fw23_Urban-Drive_Dual-Returns.json``

We will use ``SAMPLE_DATA_PCAP_PATH`` to refer to this pcap and ``SAMPLE_DATA_JSON_PATH`` to this
json in the following.  You may find it convenient to assign the paths appropriately in your console.

..
   [end-download-instructions]

Visualizing it
===============

After obtaining the recorded data, you can visualize it with the ``simple-viz`` visualizer that is
available with Ouster Python SDK (see :ref:`Python Installation <installation-python>` for
details)::
        
   $ simple-viz --pcap $SAMPLE_DATA_PCAP_PATH --meta $SAMPLE_DATA_JSON_PATH
       
You should get a view similar to:

.. figure:: /images/simple-viz.png
    :align: center

    Ouster ``simple-viz`` visualization of OS1 128 Rev 6 sample data


Congrats! You've installed and visualized with the Ouster Python SDK!