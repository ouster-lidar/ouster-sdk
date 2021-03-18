=============================
Ouster Client Python Bindings
=============================

:Project-type: lib/Python
:Description: Python bindings for ouster_client
:Maintainers: | Chris Bayruns <chris.bayruns@ouster.io>,
              | Dima Garbuzov <dima.garbuzov@ouster.io>


Summary
=======
Python bindings for ``ouster_example/ouster_client``. Currently only includes low-level wrappers
around the C++ code and helper code for reading packet data using numpy.


Building
========
Building the python client requires all the build dependencies for ``ouster_example/ouster_client``,
as well as python3 >= 3.3, pip, and pybind11 >= 2.2. You may wish to refer to the README located at 
``ouster_example`` for more on building the ouster client.

Dependencies can be provided either via system package manager or the ``vcpkg`` package manager. 
To use ``vcpkg`` set the ``CMAKE_TOOLCHAIN_FILE`` in your environment. See the `vcpkg documentation`_ 
for details.

On Ubuntu using apt::

    # install the dependencies for ouster_example/ouster_client
    sudo apt install build-essential cmake libglfw3-dev libglew-dev libeigen3-dev \
         libjsoncpp-dev libtclap-dev


    #install python3, pip, and pybind11
    sudo apt install python3-dev python3-pip pybind11-dev 

Note that the version of pybind11 packaged for Ubuntu 18.04 is too old. The location of the 
``ouster_client`` source can be specified by adding the parent directory to cmake's module search 
path.


::

   # optional, if using vcpkg -- assuming jsoncpp, eigen3 and pybind11 are installed
   export CMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake

   # specify path to ouster_util
   export CMAKE_PREFIX_PATH=/path/to/ouster_sw/lib/ouster_util

   # do an isolated build with PEP-517 support
   pip wheel --no-deps /path/to/python-client

   # or just install directly (virtualenv recommended)
   pip install /path/to/python-client

.. _vcpkg documentation: https://github.com/microsoft/vcpkg/blob/master/README.md


Developing
==========
Install in editable mode with pip using `-e` flag, or add as a path dependency for another project with 
poetry. For a faster development cycle, you can rebuild using ``python setup.py build_ext -i`` instead 
of reinstalling the package after every change. For a local debug build, you can also add the ``-g``
flag.

This package comes with a stub file providing type annotations for use with mypy. We don't currently
run mypy in CI, but all changes should typecheck. To get started, see `the official mypy
documentation`_.

.. _the official mypy documentation: https://mypy.readthedocs.io


API Docs
========
API docs are currently `available here`_. To build the html yourself, install the requirements and
use the Makefile in the ``docs`` subdirectory. Since Python's ``autodoc`` tool actually imports the
modules to extract docstrings, you'll need to do this in an env with the ``ouster-client`` package
installed::

    cd ./docs
    pip install -r requirements.txt
    make html

.. _available here: https://ouster-build.uc.r.appspot.com/ouster-client/api.html


Release
=======
See ``apps/python-commons/manylinux`` for how to build linux release artifacts.

Uploading to artifactory can be done using the ``twine`` tool. Unlike the poetry configuration, the
repo URL should not use the "/simple" siffix. For example::

    [distutils]
    index-servers =
        ouster

    [ouster]
    repository = https://ousterdev.jfrog.io/ousterdev/api/pypi/test-virtual-pypi-1/
    username = gomer

Examples
========
See ``docs/examples/example.py`` for examples in using the API.

Notes
=====
CI is currently provided by the ``sensor-client`` top-level job. Extracting the tests into this
project is WIP.

Building on macos is currently untested but should work with brew, in principle.
