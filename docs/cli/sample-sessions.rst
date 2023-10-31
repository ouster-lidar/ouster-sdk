.. _sample sessions:

Sample Sessions
===============

These sample sessions will provide sample workflows for working with pcaps, sensors, and OSFs.
Either can be done independently, but users with access to a sensor may wish to work with the sensor
first, so that they will have recorded a pcap play back in the second sample.


Sample sensor session
---------------------

Connect a sensor via Ethernet to the system with Ouster SDK installed.

.. note::

   Bear in mind that following the steps below will modify the sensor's configuration.

First we configure the sensor with standard ports, azimuth window, operating mode, and auto udp
dest:

.. code:: bash

    $ ouster-cli source <SENSOR HOSTNAME> config

You should get a return that looks like

.. code:: bash

    No config specified; using defaults and auto UDP dest:
    {
        "azimuth_window": 
        [
            0,
            360000
        ],
        "operating_mode": "NORMAL",
        "udp_port_imu": 7503,
        "udp_port_lidar": 7502
    }

Let's see what the sensor is seeing in a pretty visualizer:

.. code:: bash

    $ ouster-cli source <SENSOR HOSTNAME> viz

That looked nice! Let's record ten seconds of data to a pcap so we can view it on repeat!

.. code:: bash
    
    $ ouster-cli source <SENSOR HOSTNAME> record -s 10

That should produce screen output that looks something like:

.. code:: bash

    Connecting to <SENSOR HOSTNAME>
    Recording for up to 10.0 seconds...
    Wrote X GiB to ./OS-<SENSOR_HOSTNAME>-<LIDAR_MODE>_<DATE>.pcap

Go ahead and look in the current directory for the named pcap file and associated metadata file.

Continue to the next section, `Sample pcap session`_ to see what you can do with your new pcap file.


Sample pcap session
-------------------

If you don't have a pcap lying around or that you just recorded from a sensor, you can download one
the `OS2 bridge sample data`_ and unzip the contents.

Let's take a look at your pcap:

.. code:: bash

    $ ouster-cli source <PCAP_FILE> info

This should output something that looks like:

.. code:: bash

          Reading pcap:  [####################################]  100%          
          File size:     2247.16M
          Packets read:  85085
          Encapsulation: ETHERNET
          Capture start: 2023-02-16 22:28:58.159505
          Capture end:   2023-02-16 22:30:49.369547
          Duration:      0:01:51.210042
          UDP Streams:
              Src IP              Dst IP        Src Port        Dst Port        AF        Frag         Size        Count        
              127.0.0.1        127.0.0.1            7502            7502         4          No        33024        71182        
              127.0.0.1        127.0.0.1            7503            7503         4          No           48        13903        

That tells us the number of packets belonging to each port captured in the pcap, and the associated
size.

To visualize the pcap at 2x speed while looping back:

.. code:: bash

    $ ouster-cli source <PCAP_FILE.pcap> viz -r 2.0 -e loop 

You can check check out all the available options by typing ``--help`` after ``ouster-cli source <PCAP_FILE.pcap> viz``.

 .. _OS2 bridge sample data: https://data.ouster.io/sdk-samples/OS2/OS2_128_bridge_sample.zip
