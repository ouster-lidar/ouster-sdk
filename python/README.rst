=================
Ouster Python SDK
=================

|pypi| |python| |license_bsd| |license_freeware| |downloads|

.. |pypi| image:: https://img.shields.io/pypi/v/ouster-sdk
   :target: https://pypi.org/project/ouster-sdk/
   :alt: PyPI version

.. |python| image:: https://img.shields.io/pypi/pyversions/ouster-sdk?color=brightgreen
   :target: https://pypi.org/project/ouster-sdk/
   :alt: Supported Python versions

.. |license_bsd| image:: https://img.shields.io/badge/License-BSD%203--Clause-blue
   :target: https://github.com/ouster-lidar/ouster-sdk/blob/develop/LICENSE
   :alt: BSD 3-Clause License (source)

.. |license_freeware| image:: https://img.shields.io/badge/License-Ouster%20Freeware%20EULA-blue
   :target: https://static.ouster.dev/sdk-docs/LICENSE-freeware.txt
   :alt: Ouster Freeware EULA (build artifacts)

.. |downloads| image:: https://static.pepy.tech/badge/ouster-sdk
   :target: https://pepy.tech/project/ouster-sdk
   :alt: PyPI downloads

..
    [sdk-overview-start]

The Ouster Sensor SDK provides developers with a comprehensive set of interfaces for interacting with Ouster sensor hardware. 
The SDK includes a suite of tools, libraries, and APIs to access the full range of sensor features and generate recorded sensor data. 
It is designed to support prototyping, evaluation, and other non-safety-critical applications for both Python and C++ development environments.

Core capabilities:

* Querying and setting sensor configuration
* Recording and reading data in pcap format
* Recording and reading data in Open Sensor Format (OSF)
* Reading and buffering sensor UDP data streams reliably
* Conversion of raw data to range/signal/near_ir/reflectivity images (destaggering)
* Efficient projection of range measurements to Cartesian (x, y, z) coordinates
* Visualization of multi-beam flash lidar data
* Mapping
* Pose Optimizer
* Localization
* Zone monitor

Additionally, in Python, the SDK also provides: 

* Frame-based access to lidar data as numpy datatypes
* A responsive visualizer utility for pcap and sensor

..
    [sdk-overview-end]

Supported Platforms
-------------------

..
    [python-supported-platforms-start]

The Ouster SDK is distributed as pre-built binaries and can be easily installed using pip, the Python package installer.  
The SDK is available on the Python Package Index (`PyPI`_) for the following platforms:

- Most glibc-based Linux distributions on x86_64 and ARM64 platforms (``manylinux_2_28_x86_64``, ``manylinux_2_28_aarch64``)
- macOS >= 14 on x86_64 platforms (``macosx_14_0_x86_64``)
- macOS >= 14 on Apple Silicon for Python >= 3.8 (``macosx_14_0_arm64``)
- Windows 10/11 on x86_64 platforms (``win_amd64``)

All of our code hosted on GitHub is open source. You can find the Ouster SDK repository here: https://github.com/ouster-lidar/ouster-sdk.

Building from source is supported on:

- Ubuntu 20.04, 22.04, and Debian 11 (x86-64, aarch64)
- macOS >= 14 (arm64, x86-64)
- Windows 10/11 (x86-64)

The Ouster SDK drops languages and platforms as they exit their standard support cycle. Please
follow our `Lifecycle Policy`_ page to understand when support for a Python version, C++ compiler,
Operating System, or Sensor FW may dropped from support.

.. _PyPI: https://pypi.org/project/ouster-sdk/
.. _Lifecycle Policy: https://github.com/ouster-lidar/ouster-sdk/discussions/532

..
    [python-supported-platforms-end]

Licensing
---------

Source code in the `Ouster SDK repository`_ is licensed under the BSD 3-Clause
License. Pre-built Python wheels and other binary distributions may also include
proprietary Ouster components that are provided as freeware under additional
`Ouster Freeware EULA`_ licensing.

.. _Ouster SDK repository: https://github.com/ouster-lidar/ouster-sdk
.. _Ouster Freeware EULA: https://static.ouster.dev/sdk-docs/LICENSE-freeware.txt
