.. _cpp-building:

===================================
Building the C++ Client from Source
===================================

Building the example code requires a compiler supporting C++11 and CMake 3.1 or newer and the tclap,
jsoncpp, and Eigen3 libraries with headers installed on the system. The sample visualizer also
requires the GLFW3 and GLEW libraries.

Building on Linux / macOS
==========================

To install build dependencies on Ubuntu, run::

    sudo apt install build-essential cmake libglfw3-dev libglew-dev libeigen3-dev \
         libjsoncpp-dev libtclap-dev

On macOS, install XCode and `homebrew <https://brew.sh>`_ and run::

    brew install cmake pkg-config glfw glew eigen jsoncpp tclap

To build run the following commands::

    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release <path to ouster_example>
    make

where ``<path to ouster_example>`` is the location of the ``ouster_example`` source directory. The
CMake build script supports several optional flags::

    -DBUILD_VIZ=OFF                      Do not build the sample visualizer
    -DBUILD_PCAP=ON                      Build pcap tools. Requires libpcap and libtins dev packages
    -DBUILD_SHARED_LIBS                  Build shared libraries (.dylib or .so)
    -DCMAKE_POSITION_INDEPENDENT_CODE    Standard flag for position independent code
    -DBUILD_EXAMPLES=ON                  Build C++ examples

Building on Windows
====================

The example code can be built on Windows 10 with Visual Studio 2019 using CMake support and vcpkg
for dependencies. Follow the official documentation to set up your build environment:

* `Visual Studio <https://visualstudio.microsoft.com/downloads/>`_
* `Visual Studio CMake Support
  <https://docs.microsoft.com/en-us/cpp/build/cmake-projects-in-visual-studio?view=vs-2019>`_
* `Visual Studio CPP Support
  <https://docs.microsoft.com/en-us/cpp/build/vscpp-step-0-installation?view=vs-2019>`_
* `Vcpkg, at tag "2022.02.23" installed and integrated with Visual Studio
  <https://docs.microsoft.com/en-us/cpp/build/vcpkg?view=msvc-160#installation>`_

**Note** You'll need to run ``git checkout 2022.02.23`` in the vcpkg directory before bootstrapping
to use the correct versions of the dependencies. Building may fail unexpectedly if you skip this
step.

Don't forget to integrate vcpkg with Visual Studio after bootstrapping::

    .\vcpkg.exe integrate install

You should be able to install dependencies with::

    .\vcpkg.exe install --triplet x64-windows glfw3 glad[gl-api-33] tclap jsoncpp eigen3

After these steps are complete, you should be able to open, build and run the ``ouster_example``
project using Visual Studio:

1. Start Visual Studio.
2. When the prompt opens asking you what type of project to open click **Open a local folder** and
   navigate to the ``ouster_example`` source directory.
3. After opening the project for the first time, wait for CMake configuration to complete.
4. Make sure Visual Studio is `building in release mode`_. You may experience performance issues and
   missing data in the visualizer otherwise.
5. In the menu bar at the top of the screen, select **Build > Build All**.
6. To use the resulting binaries, go to **View > Terminal** and run, for example::

    .\out\build\x64-Release\ouster_client\ouster_client_example.exe

.. _building in release mode: https://docs.microsoft.com/en-us/visualstudio/debugger/how-to-set-debug-and-release-configurations?view=vs-2019

Running the Sample Client
==========================

Make sure the sensor is connected to the network. See "Connecting to the Sensor" in the `Software
User Manual <https://www.ouster.com/downloads>`_ for instructions and different options for network
configuration.

Navigate to ``ouster_client`` under the build directory, which should contain an executable named
``ouster_client_example``. This program will attempt to connect to the sensor, capture lidar data,
and write point clouds out to CSV files::

    ./ouster_client_example <sensor hostname> <udp data destination>

where ``<sensor hostname>`` can be the hostname (os-99xxxxxxxxxx) or IP of the sensor and ``<udp
data destingation>`` is the hostname or IP to which the sensor should send lidar data. You can also
supply ``""``, an empty string, to utilize automatic detection.

On Windows, you may need to allow the client/visualizer through the Windows firewall to receive
sensor data.
