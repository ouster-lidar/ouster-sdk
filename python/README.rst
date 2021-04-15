=================
Ouster Python SDK
=================

The Ouster Python SDK provides a high-level interface for interacting with sensor hardware and
recorded sensor data suitable for prototyping, evaluation and other non-safety-critical
applications. Example and reference code is provided for several common operations on sensor
data. The SDK includes APIs for:

- Querying and setting sensor configuration
- Recording and reading data in pcap format
- Reading and buffering sensor UDP data streams reliably
- Frame-based access to lidar data as numpy datatypes
- Conversion of raw data to range/intensity/ambient/reflectivity images (de-staggering)
- Efficiently projecting range measurements to cartesian (X, Y, Z) coordinates

See the :ref:`Quick Start Section <quickstart>` to begin working with Ouster data!

.. todo::
   Add links to examples from features list!


Supported Platforms
===================

Pre-built binaries are provided on `PyPI <https://pypi.org/>`_ for:

- Most glibc-based Linux distributions (``manylinux2010_x86_64``)
- Macos >= 10.13 (``macosx_10_13_x86_64``)
- Windows 10 (``win_amd64``)

Building from source is supported on:

- Ubuntu 18.04, 20.04, and Debian 10 (x86-64)
- Macos >= 10.13 (x86-64)
- Windows 10 (x86-64)

See :ref:`the build instructions <devel>` for requirements needed to build from source distributions
or from a clone of the repository.


Project Status
==============

The Ouster Python SDK is currently provided as an early pre-1.0 preview. The APIs are subject to
change in every release.


Contact and Links
=================

For questions about using your Ouster hardware, please contact `Ouster support
<https://ouster.atlassian.net/servicedesk/customer/portal/8>`_. For issues specific to the provided
code please use the `GitHub issue tracker <https://github.com/ouster-lidar/ouster_example/issues>`_.
