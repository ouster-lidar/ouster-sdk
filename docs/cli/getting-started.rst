Getting Started
===============

Installation
------------
The ouster sensor command line utility comes with the ouster-sdk python package.

.. code::

    pip3 install ouster-sdk

Using the cli
-------------

After installation you should have access to the ``ouster-cli`` utility, a command line interface
which is the main entry point to our internal tools. It is organized as a tree of
commands. To see what this means, open a terminal and type:

.. code::

    ouster-cli

.. note::

    On Ubuntu, if you're working outside a `virtual environment`_, you may have to add ``ouster-cli``
    to your path:

    .. code::
        
        export PATH=$PATH:/.local/bin

    We recommend using python virtual environments for all users!

That should return a menu of some possible commands, among them, ``discover``, ``source`` and ``util``.
Each of these is itself the key to a submenu of further commands. Let's try looking
at ``source``:

1. First download the sample data here :ref:`sample-data-download`.
2. Next run the following to see a list of commands

.. code::

    ouster-cli source <PCAP FILE>

You should see a few commands listed there, including ``viz`` and ``info``. To figure out how to use
them, try using ``--help``. For example, for help with the ``ouster-cli source <SAMPLE PCAP FILE>
viz`` command:

.. code:: 

    ouster-cli source <SAMPLE PCAP FILE> viz --help

Remember that you can use ``--help`` with any ``ouster-cli`` command, regardless how far down the
menu tree you are.

If you have a live sensor, you can replace the <SAMPLE PCAP FILE> with the hostname of the sensor.

To see a full list of flags and options for the ``viz`` command, run the following:

.. code:: 

    ouster-cli source <SENSOR HOSTNAME> viz --help

By default, running ``ouster-cli source <SENSOR HOSTNAME> viz`` will reconfigure the sensor to
transmit lidar packets to the host running ``ouster-cli`` and reinitialize the sensor. The ``-x``
and ``-y`` flags for the ``viz`` command disable re-initialization and packet destination
auto-configuration, respectively.

.. admonition:: Ubuntu UFW Firewall may cause: ``No packets received within 1.0s``

    On some Ubuntu setups we've observed the situations when everything is configured properly so
    that:

    - sensor is seen on the network and its Web page can be reached
    - sensor destination IP is set to the IP of the computer where data is expected
    - sensor lidar port is known (i.e. default ``7502``, or some others)
    - sensor is in ``RUNNING`` state
    - sensor lidar packets traffic is seen on the expected machine and can be recorded with
      ``tcpdump -w`` command to a pcap file (or ``Wireshark`` tools)
    - CLI comamnd ``ouster-cli source <SENSOR HOSTNAME> {info,config}`` are working properly
    - Viz ``ouster-cli source <PCAP FILE> viz`` from the ``tcpdump`` recorded pcap can be played and
      visualized
    
    But ``ouster-cli source <SENSOR HOSTNAME> viz``, or ``ouster-cli source <SENSOR HOSTNAME>
    record`` still can't receive any packets and get the following error::

        ouster.client.core.ClientTimeout: No packets received within 1.0s

    Please check your `UFW Firewall`_ settings and try to allow the UDP traffic for ``7502``
    (or whatever the **UDP Port Lidar** is set on the sensor)::

        sudo ufw allow 7502/udp

.. _UFW Firewall: https://help.ubuntu.com/community/UFW


.. _virtual environment: https://docs.python.org/3/library/venv.html

What next?
----------

You can now to use ``ouster-cli`` as you please, exploring available utilities with the handy
``---help``. If you'd prefer something more documented, you can check out our :ref:`sample sessions`
to see what an ``ouster-cli`` workflow might look like, or you can read through :ref:`common
commands`.
