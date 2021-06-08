.. _devel:

=============================
Ouster Python SDK Development
=============================

Building
========
Building the Python SDK from source requires several dependencies:

- a C++14-capable compiler
- `cmake <https://cmake.org/>`_  >= 3.5
- `eigen <https://eigen.tuxfamily.org>`_ >= 3.3
- `jsoncpp <https://github.com/open-source-parsers/jsoncpp>`_ >= 1.7
- `libtins <http://libtins.github.io/>`_ >= 3.4
- `libpcap <https://www.tcpdump.org/>`_
- `Python <https://www.python.org/>`_ >= 3.6 (with headers and development libraries)
- `pybind11 <https://pybind11.readthedocs.io>`_ >= 2.0


Linux and macos
---------------

On supported Debian-based linux systems, you can install all build dependencies by running:

.. code:: console

   $ sudo apt install build-essential cmake \
                      libeigen3-dev libjsoncpp-dev libtins-dev libpcap-dev \
                      python3-dev python3-pip pybind11-dev

On macos >= 10.13, using homebrew, you should be able to run:

.. code:: console

  $ brew install cmake eigen jsoncpp libtins python3 pybind11

After you have the system dependencies, you can build the SDK with:

.. code:: console

   # first, specify the path to the ouster_example repository
   $ export OUSTER_SDK_PATH=<PATH TO OUSTER_EXAMPLE REPO>

   # then, build an installable "wheel" package
   $ python3 -m pip wheel --no-deps $OUSTER_SDK_PATH/python

   # or just install directly (virtualenv recommended)
   $ python3 -m pip install $OUSTER_SDK_PATH/python


Windows 10
----------

On Windows 10, you'll have to install Visual Studio, Python and the `vcpkg`_ package manager and
run:

.. code:: powershell

   PS > vcpkg install eigen3 jsoncpp libtins pybind11

The currently tested vcpkg tag is ``2020.11-1``. After that, using a developer powershell prompt:

.. code:: powershell

   # first, specify the path to the ouster_example repository
   PS > $env:OUSTER_SDK_PATH=<PATH TO OUSTER_EXAMPLE>

   # point cmake to the location of vcpkg
   PS > $env:CMAKE_TOOLCHAIN_FILE=<PATH TO VCPKG REPO>/scripts/buildsystems/vcpkg.cmake

   # then, build an installable "wheel" package
   PS > py -m pip wheel --no-deps $env:OUSTER_SDK_PATH\python

   # or just install directly (virtualenv recommended)
   PS > py -m pip install $env:OUSTER_SDK_PATH\python

See the top-level README in the `Ouster Example repository`_ for more details on setting up a
development environment on Windows.

.. _vcpkg: https://github.com/microsoft/vcpkg/blob/master/README.md
.. _Ouster Example repository: https://github.com/ouster-lidar/ouster_example

Developing
==========

Install in editable mode with pip using ``pip install -e``. For a faster development cycle, you can
rebuild using ``python3 setup.py build_ext -i`` instead of reinstalling the package after every
change. For a local debug build, you can also add the ``-g`` flag.

The Ouster SDK package includes configuration for ``flake8`` and ``mypy``. To run:

.. code:: console

   # install and run flake8 linter
   $ python3 -m pip install flake8
   $ cd ${OUSTER_SDK_PATH}/python
   $ flake8

   # install and run mypy in an environment with
   $ python3 -m pip install mypy
   $ mypy src/


Running Tests
=============

To run tests while developing, install the ``pytest`` package and run it from the root of the Python
SDK package:

.. code:: console

   $ cd ${OUSTER_SDK_PATH}/python
   $ pytest

To run tests against multiple Python versions simultaneously, use the ``tox`` package:

.. code:: console

   $ cd ${OUSTER_SDK_PATH}/python
   $ tox

This will take longer, since it will build the package from a source distribution for each supported
Python version available.


Using Dockerfile
----------------

To simplify testing on multiple linux distros, a Dockerfile is included for running ``tox`` on a
variety of Debian-based distros with all packaged Python versions pre-installed. To build a test
image, run:

.. code:: console

   $ docker build ${OUSTER_SDK_PATH} -f ${OUSTER_SDK_PATH}/python/Dockerfile \
       --build-arg BASE=ubuntu:20.04 \
       -t ouster-sdk-tox \

the ``BASE`` argument will default to ``ubuntu:18.04``, but can also be set to other docker tags,
e.g. ``ubuntu:20.04`` or ``debian:10``. Then, run the container to invoke tox:

.. code:: console

   $ docker run -it --rm ouster-sdk-tox
