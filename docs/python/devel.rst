.. _devel:

=============================
Ouster Python SDK Development
=============================

Building ``ouster-sdk`` package
================================

Building the Python SDK from source requires several dependencies:

- a C++14-capable compiler
- `cmake <https://cmake.org/>`_  >= 3.5
- `eigen <https://eigen.tuxfamily.org>`_ >= 3.3
- `curl <https://curl.se/libcurl/>`_ >= 7.58
- `libtins <http://libtins.github.io/>`_ >= 3.4
- `libpcap <https://www.tcpdump.org/>`_
- `libpng <http://www.libpng.org>`_ >= 1.6
- `flatbuffers <https://flatbuffers.dev/>`_ >= 1.1
- `libglfw3 <https://www.glfw.org/>`_ >= 3.2
- `python <https://www.python.org/>`_ >= 3.8 (with headers and development libraries)
- `pybind11 <https://pybind11.readthedocs.io>`_ >= 2.0

The Python SDK source is available `on the Ouster Github <https://github.com/ouster-lidar/ouster-sdk>`_. You should clone the whole project.

Linux and macOS
---------------

On supported Debian-based Linux systems, you can install all build dependencies by running:

.. code:: console

   $ sudo apt install build-essential cmake \
                      libeigen3-dev libtins-dev libpcap-dev \
                      python3-dev python3-pip libcurl4-openssl-dev \
                      libglfw3-dev libpng-dev libflatbuffers-dev libssl-dev

On macOS >= 11, using Homebrew, you should be able to run:

.. code:: console

  $ brew install cmake eigen curl libtins python3 glfw libpng flatbuffers

After you have the system dependencies, you can build the SDK with:

.. code:: console

   # first, specify the path to the ouster-sdk repository
   $ export OUSTER_SDK_PATH=<PATH TO OUSTER_SDK REPO>

   # make sure you have an up-to-date version of pip and setuptools installed
   $ python3 -m pip install --user --upgrade pip setuptools

   # install pybind11
   $ python3 -m pip install pybind11

   # then, build an installable "wheel" package
   $ python3 -m pip wheel --no-deps $OUSTER_SDK_PATH/python

   # or just install directly (virtualenv recommended)
   $ python3 -m pip install $OUSTER_SDK_PATH/python


.. note::

   We recommend to always use `Virtual Environment`_ for python development.

.. _Virtual Environment: https://docs.python.org/3/library/venv.html#module-venv


Windows 10
----------

On Windows 10, you'll have to install the Visual Studio 2017 Build Tools, Python and the `vcpkg`_
package manager and run:

.. code:: powershell

   PS > vcpkg install --triplet=x64-windows curl eigen3 libtins glfw3 glad[gl-api-33] libpng flatbuffers

The currently tested vcpkg tag is ``2024.04.26``. After that, using a developer powershell prompt:

.. code:: powershell

   # first, specify the path to the ouster-sdk repository
   PS > $env:OUSTER_SDK_PATH="<PATH TO OUSTER_SDK>"

   # point cmake to the location of vcpkg (make sure to use an absolute path)
   PS > $env:CMAKE_TOOLCHAIN_FILE="<PATH TO VCPKG REPO>\scripts\buildsystems\vcpkg.cmake"

   # set the correct vcpkg triplet
   PS > $env:VCPKG_TARGET_TRIPLET="x64-windows"
   
   # set build options related to the compiler
   PS > $env:CMAKE_GENERATOR_PLATFORM="x64"
   PS > $env:CMAKE_GENERATOR="Visual Studio 15 2017"

   # install pybind11
   PS > py -m pip install pybind11 ninja
   
   # then, build an installable "wheel" package
   PS > py -m pip wheel --no-deps "$env:OUSTER_SDK_PATH\python"

   # or just install directly (virtualenv recommended)
   PS > py -m pip install "$env:OUSTER_SDK_PATH\python"

See the top-level README in the `Ouster SDK repository`_ for more details on setting up a
development environment on Windows.

.. _vcpkg: https://github.com/microsoft/vcpkg/blob/master/README.md
.. _Ouster SDK repository: https://github.com/ouster-lidar/ouster-sdk


Developing
==========

Install in editable mode with pip using ``pip install -e``. For a faster development cycle, you can
rebuild using ``python3 setup.py build_ext -i`` instead of reinstalling the package after every
change. For a local debug build, you can also add the ``-g`` flag.

The Ouster SDK package includes configuration for ``flake8`` and ``mypy``. To run:

.. code:: console

   # install pybind11
   $ python3 -m pip install pybind11
   
   # install and run flake8 linter
   $ python3 -m pip install flake8
   $ cd ${OUSTER_SDK_PATH}/python
   $ python3 -m flake8

   # install and run mypy in an environment with
   $ python3 -m pip install mypy
   $ python3 -m mypy src/


Running Tests
=============

To run tests while developing, install the ``pytest`` package and run it from the root of the Python
SDK package:

.. code:: console

   $ cd ${OUSTER_SDK_PATH}/python
   $ python3 -m pytest

To run interactive :class:`.viz.PointViz` tests, use ``--interactive`` argument:

.. code:: console

   $ cd ${OUSTER_SDK_PATH}/python
   $ python3 -m pytest --interactive

To run tests against multiple Python versions simultaneously, use the ``tox`` package:

.. code:: console

   $ cd ${OUSTER_SDK_PATH}/python
   $ python3 -m tox

This will take longer, since it will build the package from a source distribution for each supported
Python version available.

