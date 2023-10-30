
.. _installation:

=======================
Ouster SDK Installation
=======================

Please proceed to the instructions for your programming language and platform of choice.

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

The Ouster Python SDK binary packages require Python >= 3.7 and pip >= 19.0 on most platforms. On
Ubuntu 18.04, the default Python 3 version is is 3.6, so you'll have to install and use
``python3.7`` explicitly. On Apple M1, you'll need need Python >= 3.8.

.. note::

    Using a virtual environment with the Ouster Python SDK is recommended.
    Users newer to Python should read the official `venv instructions`_ and
    ensure that they upgrade pip *after* activating their venv. If you're using
    venv on Windows, you'll want to use ``python`` and ``pip`` instead of ``py
    -3`` and ``py -3 -m pip`` in the following Powershell snippets.

.. note::

    Python 3 when installed with macOS Developer Tools uses LibreSSL 2.8.3 (or
    an older version.) OusterSDK, like many Python 3-compatible packages,
    requires urllib3 which is not compatible with LibreSSL and requires OpenSSL
    1.1.1 or newer. To account for this, macOS users should install an official
    distribution of Python 3 or one provided by Homebrew, as these use OpenSSL
    1.1.1 or newer. In either case, installing OusterSDK into a Python 3
    virtual enviroment is still recommended.

If you're using an unsupported platform like a non-glibc-based Linux distribution, or wish to modify
the Ouster Python SDK, you will need to build from source. See the `build instructions`_ for
requirements needed to build from a source distribution or from a clone of the repository.

To install on :ref:`supported platforms<supported-platforms>`, first make sure you have the latest
version of pip:

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


ROS
===
Ouster ROS driver code has been moved out to a separate GitHub repository. To get started using the
driver follow the instructions provided on the repository's main page: https://github.com/ouster-lidar/ouster-ros
