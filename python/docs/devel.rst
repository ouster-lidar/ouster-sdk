.. _devel-instructions:

=============================
Ouster Python SDK Development
=============================

.. todo::
   Update devel.rst for external consumption

Building
========
Building the python client requires all the build dependencies for ``ouster_example/ouster_client``,
as well as python3 >= 3.3, pip, and pybind11 >= 2.2. You may wish to refer to the README located at 
``ouster_example`` for more on building the ouster client.

Dependencies can be provided either via system package manager or the ``vcpkg`` package manager. 
To use ``vcpkg`` set the ``CMAKE_TOOLCHAIN_FILE`` in your environment. See the `vcpkg documentation`_ 
for details.

On Ubuntu using apt::

    # install the dependencies for ouster_example/ouster_client and ouster_example/ouster_pcap
    sudo apt install build-essential cmake libglfw3-dev libglew-dev libeigen3-dev \
         libjsoncpp-dev libtclap-dev libtins


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


Running Tests
=============

Ouster SDK Python client has unit tests that can be run with ``pytest`` and
``tox`` to test against multiple python innterpreters versions.

::

   cd ${OUSTER_SDK_PATH}/python
   tox

By default ``tox`` is configured to run tests for Pythons versions
``py{36,37,38,39}`` (set in ``tox.ini``)

Additionally ``tox`` has configurations for testing pre-build wheels
``py{36,37,38,39}-use_wheels`` testenv. The location of the wheels should be passed
with ``WHEELS_DIR`` env var:

::

   WHEELS_DIR=/pre/build/wheels/dir tox -e "py{36,37,38,39}-use_wheels"


Using Dockerfile.tox
--------------------

For platform independent tests/validation you can use ``Dockerfile.tox`` that
pre-configured with all ``ouster-sdk`` build dependencies and can run tests on
a subset of recent Debian based distros (build arg BASE can ``ubuntu:18.04``,
``ubuntu:20.04`` and ``debian:10``)

Build Docker image for running ``tox`` tests:

::

   docker build ${OUSTER_SDK_PATH} -t ouster_python_client \
      -f ${OUSTER_SDK_PATH}/python/Dockerfile.tox \
      --build-arg BUILD_UID=$(id -u) \
      --build-arg BUILD_GID=$(id -g)

Run tests with default tox command (only ``py{36,37,38,39``):

::

   docker run -it --rm \
      -v ${OUSTER_SDK_PATH}:/opt/ws/ouster_sdk \
      ouster_python_client

Because not all Python interpreter verions included in all stock distros, here
is the table of BASE/pyXX versions that is covered byt the ``Dockerfile.tox``:

- ``ubuntu:18.04`` x ``py{36,37,38}``
- ``ubuntu:20.04`` x ``py{38,39}``
- ``debian:10`` x ``py{37}``
