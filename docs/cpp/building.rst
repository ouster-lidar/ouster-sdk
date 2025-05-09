.. _cpp-building:

===================================
Building the C++ Client from Source
===================================

Building the example code requires a compiler supporting C++14 and CMake 3.1 or newer and the
Eigen3 and tins libraries with headers installed on the system. The sample visualizer also
requires the GLFW3 and GLEW libraries.

The C++ example code is available `on the Ouster Github
<https://github.com/ouster-lidar/ouster-sdk>`_. Follow the instructions for cloning the project.

Building on Linux / macOS
=========================

To install build dependencies on Ubuntu:20.04+, run:

.. code:: console

   $ sudo apt install build-essential cmake libeigen3-dev libcurl4-openssl-dev \
                      libtins-dev libpcap-dev libglfw3-dev libpng-dev \
                      libflatbuffers-dev

You may also install curl with a different ssl backend, for example libcurl4-gnutls-dev or
libcurl4-nss-dev.

On macOS, install XCode and `homebrew <https://brew.sh>`_ and run:

.. code:: console

   $ brew install cmake pkg-config eigen curl libtins glfw libpng flatbuffers

To build on macOS and Ubuntu:20.04+ run the following commands:

.. code:: console

   $ mkdir build
   $ cd build
   $ cmake  <path to ouster-sdk/CMakeLists.txt> -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON
   $ cmake --build . -- -j$(nproc)

where ``<path to ouster-sdk>`` is the location of the ``ouster-sdk`` source directory. The
CMake build script supports several optional flags. Add any of the following to override the
defaults:

.. code:: console

   -DBUILD_VIZ=OFF                    # Do not build the sample visualizer
   -DBUILD_PCAP=OFF                   # Do not build pcap tools
   -DBUILD_OSF=OFF                    # Do not build OSF lib
   -DBUILD_EXAMPLES=ON                # Build C++ examples
   -DBUILD_TESTING=ON                 # Build tests
   -DBUILD_SHARED_LIBRARY=ON          # Build the shared library and binary artifacts


Building on Windows
===================

The example code can be built on Windows 10 with Visual Studio 2019 using CMake support and vcpkg
for dependencies. Follow the official documentation to set up your build environment:

* `Visual Studio <https://visualstudio.microsoft.com/downloads/>`_
* `Visual Studio CMake Support
  <https://docs.microsoft.com/en-us/cpp/build/cmake-projects-in-visual-studio?view=vs-2019>`_
* `Visual Studio CPP Support
  <https://docs.microsoft.com/en-us/cpp/build/vscpp-step-0-installation?view=vs-2019>`_
* `Vcpkg, at tag "2024.04.26" installed and integrated with Visual Studio
  <https://docs.microsoft.com/en-us/cpp/build/vcpkg?view=msvc-160#installation>`_

**Note** You'll need to run ``git checkout 2024.04.26`` in the vcpkg directory before bootstrapping
to use the correct versions of the dependencies. Building may fail unexpectedly if you skip this
step.

Don't forget to integrate vcpkg with Visual Studio after bootstrapping:

.. code:: powershell

   PS > .\vcpkg.exe integrate install

You should be able to install dependencies with

.. code:: powershell

   PS > .\vcpkg.exe install --triplet x64-windows eigen3 curl libtins glfw3 libpng flatbuffers

After these steps are complete, you should be able to open, build and run the ``ouster-sdk``
project using Visual Studio:

1. Start Visual Studio.
2. When the prompt opens asking you what type of project to open click **Open a local folder** and
   navigate to the ``ouster-sdk`` source directory.
3. After opening the project for the first time, wait for CMake configuration to complete.
4. Make sure the ``Desktop development with C++`` is installed. If not, install it using the Search bar
   on the top of the screen.
5. Make sure Visual Studio is `building in release mode`_. You may experience performance issues and
   missing data in the visualizer otherwise.
6. In the menu bar at the top of the screen, select **Build > Build All**.
7. To use the resulting binaries, go to **View > Terminal** and run, for example:

.. code:: powershell

   .\out\build\x64-Release\examples\client_example.exe

.. _building in release mode: https://docs.microsoft.com/en-us/visualstudio/debugger/how-to-set-debug-and-release-configurations?view=vs-2019

Running the Sample Client
=========================

Make sure the sensor is connected to the network. See "Connecting to the Sensor" in the `Software
User Manual <https://www.ouster.com/downloads>`_ for instructions and different options for network
configuration.

Navigate to ``examples`` under the build directory, which should contain an executable named
``client_example``. This program will attempt to connect to the sensor, capture lidar data, and
write point clouds out to CSV files:

.. code:: console

   $ ./client_example <sensor hostname> <udp data destination>

where ``<sensor hostname>`` can be the hostname (os-99xxxxxxxxxx) or IP of the sensor and ``<udp
data destination>`` is the hostname or IP to which the sensor should send lidar data. You can also
supply ``""``, an empty string, to utilize automatic detection.

On Windows, you may need to allow the client/visualizer through the Windows firewall to receive
sensor data.

Building Against The Library
============================

Navigate to ``examples`` under the ouster-sdk source directory, which should contain several linux 
example folders building against the sdk library. To run each use the example.bash script.

1. compiled_in_linking_example - Compile ouster_sdk as a sub-project under a larger codebase.
2. static_linking_example - Use installed static libs of ouster_sdk under a larger codebase.
3. shared_linking_example - Use installed shared libs of ouster_sdk under a larger codebase.

