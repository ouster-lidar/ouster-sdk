=================
Ouster Python SDK
=================

The Ouster Python SDK provides a high-level interface for interacting with sensor hardware and
recorded sensor data suitable for prototyping, evaluation, and other non-safety-critical
applications. Example and reference code is provided for several common operations on sensor
data. The SDK includes APIs for:

- Querying and setting sensor configuration
- Recording and reading data in pcap format
- Reading and buffering sensor UDP data streams reliably
- Frame-based access to lidar data as numpy datatypes
- Conversion of raw data to range/signal/near_ir/reflectivity images (de-staggering)
- Efficiently projecting range measurements to cartesian (X, Y, Z) coordinates

.. _supported platforms:


Supported Platforms
-------------------

Pre-built binaries are provided on `PyPI`_ for x86_64 platforms including:

- Most glibc-based Linux distributions (``manylinux2010_x86_64``)
- macOS >= 10.13 (``macosx_10_13_x86_64``)
- Windows 10 (``win_amd64``)

Building from source is supported on:

- Ubuntu 18.04, 20.04, and Debian 10 (x86-64)
- macOS >= 10.13 (x86-64)
- Windows 10 (x86-64)

.. _PyPI: https://pypi.org/project/ouster-sdk/


Installation
------------

The Ouster Python SDK requires Python >= 3.6 and pip >= 19.0. To install, use ``pip`` to grab the
``ouster-sdk`` package. See the `quick start`_ section of the documentation for more details and to
begin working with Ouster data!

If you're running ARM, a non-glibc-based linux distribution, or wish to modify the Ouster Python
SDK, you will need to build from source. See the `build instructions`_ for requirements needed to
build from a source distribution or from a clone of the repository.

.. _quick start: https://static.ouster.dev/sdk-docs/quickstart.html
.. _build instructions: https://static.ouster.dev/sdk-docs/devel.html


Status and Contact Info
-----------------------

The Ouster Python SDK is currently provided as an early pre-1.0 preview. The APIs are subject to
change in every release.

For questions about using your Ouster hardware, please contact `Ouster support`_. For issues
specific to the provided code please use the `GitHub issue tracker`_.

.. _Ouster support: https://ouster.atlassian.net/servicedesk/customer/portal/8
.. _Github issue tracker: https://github.com/ouster-lidar/ouster_example/issues
