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

You can download sample data from the `sensor documentation`_ by clicking through the Ouster Data
App links and using the Download button. After download, you should have two files, a ``.pcap`` file
and a ``.json`` file.

We will use ``SAMPLE_DATA_PCAP_PATH`` to refer to this pcap and ``SAMPLE_DATA_JSON_PATH`` to this
json in the following.  You may find it convenient to assign the paths appropriately in your
console.

.. _sensor documentation: https://static.ouster.dev/sensor-docs/#sample-data

..
   [end-download-instructions]

.. note::
    
    All Ouster sample data is provided under the `CC BY-NC-SA license
    <https://creativecommons.org/licenses/by-nc-sa/2.0/>`_, whether obtained
    through the above links or from the Ouster website.

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
