=============================
Ouster Client Python Bindings
=============================

:Maintainers: Chris Bayruns <chris.bayruns@ouster.io>, Dima Garbuzov <dima.garbuzov@ouster.io>
:Description: Python bindings for ouster_client
:Project-type: lib/Python


Summary
=======
Python bindings for ``ouster_example/ouster_client``. Currently only includes low-level wrappers
around the C++ code and helper code for reading packet data using numpy.


Building
========
Requires all the build dependencies for ``ouster_example/ouster_client`` and, additionally, pybind11
>= 2.2 and python >= 3.3. Note: the version of pybind11 packaged for Ubuntu 18.04 is too old. The
location of the ``ouster_client`` source can be specified by adding the parent directory to cmake's
module search path.

Dependencies can be provided either via system package manager or the ``vcpkg`` package manager. To
use ``vcpkg`` set the ``CMAKE_TOOLCHAIN_FILE`` in your environment. See the `vcpkg documentation`_
for details.

::

   # optional, if using vcpkg -- assuming jsoncpp, eigen3 and pybind11 are installed
   export CMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake

   # specify parent directory of ouster_client
   export CMAKE_PREFIX_PATH=/path/to/ouster_example

   # do an isolated build with PEP-517 support
   pip wheel --no-deps /path/to/python-client

   # or just install directly (virtualenv recommended)
   pip install /path/to/python-client

.. _vcpkg documentation: https://github.com/microsoft/vcpkg/blob/master/README.md


Developing
==========
Install in editable mode with pip, or add as a path dependency for another project with poetry. For
a faster development cycle, you can rebuild using ``python setup.py build_ext -i`` instead of
reinstalling the package after every change. For a local debug build, you can also add the ``-g``
flag.


Release
=======
See ``apps/python-commons/manylinux`` for how to build linux release artifacts.

Uploading to artifactory can be done using the ``twine`` tool. Unlike the poetry configuration, the
repo URL should not use the "/simple" siffix. For example:

::

    [distutils]
    index-servers =
        ouster

    [ouster]
    repository = https://ousterdev.jfrog.io/ousterdev/api/pypi/test-virtual-pypi-1/
    username = gomer


Notes
=====
CI is currently provided by the ``shared_sw`` top-level job. Extracting the tests into this project
is WIP.

Building on macos is currently untested but should work with brew, in principle.
