
.. _installation:

Ouster SDK Installation
=======================

This section contains the installation instructions for the Ouster SDK on suported platforms in previous page.


.. _installation-python:

.. _supported-platforms:

The Ouster Python SDK is provided as pre-built binaries on PyPI. It's recommended to install it in a virtual environment
The binary packages require Python ``>= 3.8; <= 3.13`` and pip ``>= 19.0`` on most platforms.

To install, first ensure pip is up to date, then install the ouster-sdk package using the command below:

.. note::

    Using a virtual environment with the Ouster Python SDK is recommended.
    Users newer to Python should read the official `venv instructions`_ and
    ensure that they upgrade pip *after* activating their venv. If you're using
    venv on Windows, you'll want to use ``python3`` and ``pip3`` instead of ``py
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
the Ouster Python SDK, you will need to build from source.

See the development instructions for building the :doc:`Python code <../development/python>` or
:doc:`C++ code <../development/cpp>` from source using the links.

.. _ouster-sdk-installation-commands:

To install on :ref:`supported platforms<supported-platforms>`, first make sure you have the latest
version of pip and setuptools:

.. tab-set::
   :sync-group: os-commands

   .. tab-item:: Linux/macOS
      :sync: unix

      .. code::

        $ python3 -m pip install --upgrade pip setuptools
   
   .. tab-item:: Windows x64
      :sync: windows

      .. code::

        PS >  py -3 -m pip install --upgrade pip setuptools

Now that your Python environment has an up-to-date pip, you can install latest ouster-sdk using the command below:

.. tab-set::
   :sync-group: os-commands

   .. tab-item:: Linux/macOS
      :sync: unix

      .. only:: not internal_docs

         To upgrade:

      .. parsed-literal::

        $ |install_unix_upgrade|

      .. only:: not internal_docs

         For a fresh install:

         .. parsed-literal::

           $ |install_unix_fresh|
   
   .. tab-item:: Windows x64
      :sync: windows

      .. only:: not internal_docs

         To upgrade:

      .. parsed-literal::

        PS > |install_windows_upgrade|

      .. only:: not internal_docs

         For a fresh install:

         .. parsed-literal::

           PS > |install_windows_fresh|

To check that you've successfully installed the latest version of the Ouster Python SDK, run the
following command and make sure that the ``ouster-sdk`` package is included in the output:

.. tab-set::
   :sync-group: os-commands

   .. tab-item:: Linux/macOS
      :sync: unix

      .. code::

        $ python3 -m pip list

   .. tab-item:: Windows x64
      :sync: windows

      .. code::

        PS > py -3 -m pip list


You should see something like:

.. parsed-literal::

        ouster-sdk                    \ |release|\

Proceed to the next section to visually confirm your installation.

.. _venv instructions: https://packaging.python.org/guides/installing-using-pip-and-virtual-environments/#creating-a-virtual-environment