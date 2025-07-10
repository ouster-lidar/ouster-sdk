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
- `python <https://www.python.org/>`_ >= 3.8; <= 3.13 (with headers and development libraries)
- `pybind11 <https://pybind11.readthedocs.io>`_ >= 2.0

The Python SDK source is available `on the Ouster Github <https://github.com/ouster-lidar/ouster-sdk>`_. You should clone the whole project.


Linux and macOS
---------------

It is now recommended to use ``vcpkg`` to install dependencies across all platforms. 
The list of required dependencies is defined in the ``vcpkg.json`` file located at the root of the repository. 
vcpkg manifest mode is used to manage ouster-sdk dependencies in a consistent and reproducible way.

To install ``vcpkg``, please follow the instructions in the `vcpkg documentation`_.

.. _vcpkg documentation: https://learn.microsoft.com/en-us/vcpkg/get_started/get-started?pivots=shell-bash

The ``setup.py`` sets the following environment variables to point to the vcpkg installation,
please make sure to update them if want to update location of ``vcpkg.json`` file:

.. code:: console

   $ export VCPKG_MANIFEST_DIR=<PATH TO OUSTER_SDK REPO>
   $ export VCPKG_MANIFEST_MODE=ON

As an alternative, you can install the dependencies manually using following instructions:

On supported Debian-based Linux systems,

.. code:: console

   $ sudo apt install build-essential cmake \
                      libeigen3-dev libtins-dev libpcap-dev \
                      python3-dev python3-pip libcurl4-openssl-dev \
                      libglfw3-dev libpng-dev libflatbuffers-dev libssl-dev \
                      libceres-dev libtbb-dev \
                      robin-map-dev

On macOS >= 11, using Homebrew, you should be able to run:

.. code:: console

  $ brew install cmake eigen curl libtins python3 glfw libpng flatbuffers ceres-solver robin-map

You can build the SDK with:

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

As mentioned, ``vcpkg`` is the recommended package manager for ```ouster-sdk``.

Please follow the instructions in the `vcpkg documentation powershell`_ to install vcpkg.

Additionally, ensure that the following are installed on your system:

- Visual Studio 2017 Build Tools

- Python

After that, using a developer powershell prompt:

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

Optionally, if you want to update the default settings of directory for the manifest file or disable manifest mode, 
update the environment variables below: 

.. code:: powershell
   
   PS > $env:VCPKG_MANIFEST_DIR=<PATH TO OUSTER_SDK REPO>
   PS > $env:VCPKG_MANIFEST_MODE=ON

Then, you can build the SDK with:

.. code:: powershell

   # then, build an installable "wheel" package
   PS > py -m pip wheel --no-deps "$env:OUSTER_SDK_PATH\python"

   # or just install directly (virtualenv recommended)
   PS > py -m pip install "$env:OUSTER_SDK_PATH\python"

See the top-level README in the `Ouster SDK repository`_ for more details on setting up a
development environment on Windows.

.. _vcpkg documentation powershell: https://learn.microsoft.com/en-us/vcpkg/get_started/get-started?pivots=shell-powershell
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

