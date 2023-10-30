=================
Ouster Python SDK
=================

..
    [sdk-overview-start]

The Ouster Sensor SDK provides developers interfaces for interacting with sensor hardware and
recorded sensor data suitable for prototyping, evaluation, and other non-safety-critical
applications in Python and C++. Example and reference code is provided for common operations on
sensor data in both languages. The SDK includes APIs for:

* Querying and setting sensor configuration
* Recording and reading data in pcap format
* Reading and buffering sensor UDP data streams reliably
* Conversion of raw data to range/signal/near_ir/reflectivity images (destaggering)
* Efficient projection of range measurements to Cartesian (x, y, z) coordinates
* Visualization of multi-beam flash lidar data

Additionally, in Python, the SDK also provides: 

* Frame-based access to lidar data as numpy datatypes
* A responsive visualizer utility for pcap and sensor

..
    [sdk-overview-end]

Supported Platforms
-------------------

..
    [python-supported-platforms-start]

Pre-built binaries are provided on `PyPI`_ for the following platforms:

- Most glibc-based Linux distributions on x86_64 and ARM64 platforms (``manylinux2010_x86_64``,
  ``manylinux2014_aarch64``)
- macOS >= 10.15 on x86_64 platforms (``macosx_10_15_x86_64``)
- macOS >= 11.0 on Apple M1 for Python >= 3.8 (``macosx_11_0_arm64``)
- Windows 10 on x86_64 platforms (``win_amd64``)

Building from source is supported on:

- Ubuntu 20.04, 22.04, and Debian 11 (x86-64, aarch64)
- macOS >= 10.15 (x86-64), >= 11.0 (arm64)
- Windows 10 (x86-64)

The Ouster SDK drops languages and platforms as they exit their standard support cycle. Please
follow our `Lifecycle Policy`_ page to understand when support for a Python version, C++ compiler,
Operating System, or Sensor FW may dropped from support.

.. _PyPI: https://pypi.org/project/ouster-sdk/
.. _Lifecycle Policy: https://github.com/ouster-lidar/ouster_example/discussions/532

..
    [python-supported-platforms-end]
