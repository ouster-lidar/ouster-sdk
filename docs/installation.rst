
.. _installation:

=======================
Ouster SDK Installation
=======================

Please proceed to the instructions for your language and platform of choice. Newer users may find the Python SDK more approachable.

.. contents::
   :local:
   :depth: 2


.. _installation-python:

Python
========

.. _supported-platforms:

Supported Platforms
-------------------

.. include:: /../python/README.rst
    :start-after: [python-supported-platforms-start]
    :end-before: [python-supported-platforms-end]

Installation
--------------

The Ouster Python SDK requires Python >= 3.6 and pip >= 19.0. 

.. note::

    Using a virtual environment with the Ouster Python SDK is recommended. Users newer to Python should
    read the official `venv instructions`_ and ensure that they upgrade pip *after* activating their
    venv. If you're using venv on Windows, you'll want to use ``python`` and ``pip`` instead of ``py
    -3`` and ``py -3 -m pip`` in the following Powershell snippets.

.. note::

    If you're running a non-glibc-based linux distribution, or wish to modify the Ouster Python
    SDK, you will need to build from source. See the `build instructions`_ for requirements needed to
    build from a source distribution or from a clone of the repository.

.. note::
    
    Apple M1 users should be aware that due to numpy support limitations they will need to use Python
    >=3.8.

To install on :ref:`supported platforms<supported-platforms>`, please upgrade your pip:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m pip install --upgrade pip 

    .. code-tab:: powershell Windows x64
        
        PS > py -3 -m pip install --upgrade pip


Now that your Python environment has an up-to-date pip, you can install ouster-sdk:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m pip install 'ouster-sdk[examples]'

    .. code-tab:: powershell Windows x64

        PS > py -3 -m pip install 'ouster-sdk[examples]'


.. note::
    
    While the optional examples component is not required for ouster-sdk, we recommend installing it
    so you can run the various :doc:`examples </python/examples/index>`.
 

To check that you've successfully installed the latest version of the Ouster Python SDK, run the
following command and make sure that the ``ouster-sdk`` package is included in the output:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m pip list

    .. code-tab:: powershell Windows x64

        PS > py -3 -m pip list


You should see something like:

.. parsed-literal::

        ouster-sdk                    \ |release|\

Now you can visually confirm your installation with :doc:`Download and Visualize Sample Data <sample-data>`!

.. _venv instructions: https://packaging.python.org/guides/installing-using-pip-and-virtual-environments/#creating-a-virtual-environment

.. _build instructions: https://static.ouster.dev/sdk-docs/python/devel.html


What's Next
-----------

To get a feel for working with the Ouster Sensor Python API, check out the following sections:

- :doc:`Download and Visualize Sample Data <sample-data>`
- :doc:`python/quickstart`
- :doc:`python/viz/viz-api-tutorial`


C++
===

The Ouster C++ SDK currently must be built and installed from sources.
Please refer to :doc:`/cpp/building`.


ROS1
====

A sample ROS1 driver is provided. It must be built from source. Please refer to :doc:`/ros/index`.
