=============================
 Ouster PCAP Python Bindings
=============================

:Maintainers: Chris Bayruns <chris.bayruns@ouster.io>, Dima Garbuzov <dima.garbuzov@ouster.io>
:Description: Python bindings for ouster_pcap
:Project-type: lib/Python


Summary
=======
Python bindings for ``ouster_pcap``.


Building
========
Requires all the build dependencies for ``ouster_example/ouster_client`` and, additionally, pybind11
>= 2.2 and python >= 3.3. Note: the version of pybind11 packaged for Ubuntu 18.04 is too old. The
location of the ``ouster_client`` source can be specified by adding the parent directory to cmake's
module search path. The location of the ``ouster_pcap`` source can be specified by adding the parent
directory to cmake's module search path.

Dependencies can be provided either via system package manager or the ``vcpkg`` package manager. To
use ``vcpkg`` set the ``CMAKE_TOOLCHAIN_FILE`` in your environment. See the `vcpkg documentation`_
for details.

::

   # optional, if using vcpkg -- assuming jsoncpp, eigen3 and pybind11 are installed
   export CMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake

   
   # specify parent directory of ouster_client and ouster_pcap on linux
   export CMAKE_PREFIX_PATH=/path/to/ouster_example:/path/to/ouster_pcap

   # do an isolated build with PEP-517 support
   pip wheel --no-deps /path/to/python-pcap

   # or just install directly (virtualenv recommended)
   pip install /path/to/python-pcap

.. _vcpkg documentation: https://github.com/microsoft/vcpkg/blob/master/README.md


Notes
=====
CI is currently provided by the ``shared_sw`` top-level job. Extracting the tests into this project
is WIP.

Outstanding TODOs include:
- Official linux build should be done with vcpkg as well, so jsoncpp can be statically linked,
  avoiding dependencies on system shared libraries
- Use the static triplet for windows builds as well. Follow the `instructions here`_
- Building on macos is currently untested, but should work with brew, in principle
- Source distribution support is unfinished
- Need to look into `python manylinux`_ builds

.. _instructions here: https://devblogs.microsoft.com/cppblog/vcpkg-updates-static-linking-is-now-available/
.. _python manylinux: https://github.com/pypa/manylinux/blob/master/README.rst
