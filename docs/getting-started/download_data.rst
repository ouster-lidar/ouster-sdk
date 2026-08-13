Download Data
=============


.. _sample-data-download:

..
   [start-download-instructions]

Ouster provides multiple datasets via Ouster Studio to facilitate your learning experience with spatial perception. 

You can download some commonly used datasets to follow along the tutorials by clicking the `Ouster Studio`_ links 
and using the Download button for the appropriate format such as pcap or osf.  


.. _Ouster Studio: https://studio.ouster.com/public/sample_library/public_collection


.. rst-class:: download-data-table

==================================== ==================== ================================================================
Sample dataset                       Specifications        Recording
==================================== ==================== ================================================================
OS0 128 Rev 07 Urban Drive           2048x10 Dual Returns  `Download <https://studio.ouster.com/share/RRSY7O1GWJ332ERO>`__
OS1 128 Rev 07 Urban Drive           2048x10 Dual Returns  `Download <https://studio.ouster.com/share/VVX59BLMFXPLVPI5>`__
OS0 128 Rev7 Urban Drive             1024x10 Dual Returns  `Download <https://studio.ouster.com/share/ONK2420ES2S0BF8Q>`__
OS1 128 Rev7 Urban Drive             1024x10 Dual Returns  `Download <https://studio.ouster.com/share/UND0A07N9DZTFJXW>`__
OS2 128 Rev7 Highway Drive           Dual Returns          `Download <https://studio.ouster.com/share/ENOD8SQUHJKQF62T>`__
OS2 128 Rev7 Highway Drive           Dual Returns          `Download <https://studio.ouster.com/share/ENOD8SQUHJKQF62T>`__
OS1 128 Rev7 Loop with GPS Drive     2048x10 Dual Returns  `Download <https://studio.ouster.com/share/T29BEI2EIL55T648>`__
==================================== ==================== ================================================================


If you download the data in pcap format, you should have two files, a ``.pcap`` file and a ``.json`` file.

The downloaded pcap file contains lidar and imu packets captured from the network. You can read
more about the `IMU Data Format`_ and `Lidar Data Format`_ in the Ouster Sensor Documentation. The
JSON file contains metadata queried from the sensor TCP interface necessary for interpreting
the packet data.

We will use ``SAMPLE_DATA_PCAP_PATH`` to refer to this pcap and ``SAMPLE_DATA_JSON_PATH`` to this
json in the following section.  You may find it convenient to assign the paths appropriately in your
console.

In your open python session, save the two paths to variables:

.. code:: python

   >>> pcap_path = '{SAMPLE_DATA_PCAP_PATH}'
   >>> metadata_path = '{SAMPLE_DATA_JSON_PATH}'


.. _sensor documentation: https://docs.ouster.com/sensor-docs
.. _Lidar Data Format: https://docs.ouster.com/sensor-docs/docs/sensor-data/lidar-data
.. _IMU Data Format: https://docs.ouster.com/sensor-docs/docs/sensor-data/imu-data

..
   [end-download-instructions]

.. tip::
    
    All Ouster sample data is provided under the `CC BY-NC-SA license`_, whether obtained
    through the above links or from the Ouster website.

.. _CC BY-NC-SA license: https://creativecommons.org/licenses/by-nc-sa/2.0/
