.. title:: Ouster SDK Installation

.. _installation:

======================================
SDK Installation
======================================

Various ways to install and use it, depending on you platform of choice.

.. contents::
   :local:
   :depth: 1


.. _installation-python:

Python
========

Installation
--------------

.. todo::
   Moved from ``python/instalation.rst``. Required reading/fixing.

The Ouster Python SDK requires Python >= 3.6 and pip >= 19.0. 

Using a virtual environment with the Ouster Python SDK is recommended. Users newer to Python should
read the official `venv instructions`_ and ensure that they upgrade pip *after* activating their
venv. If you're using venv on Windows, you'll want to use ``python`` and ``pip`` instead of ``py
-3`` and ``py -3 -m pip`` in the following Powershell snippets.

.. todo::
   Supported platforms parts not here anymore. They are in `ouster-sdk/python/README`. If we need
   them here we can include with ``include`` directive and ``start-after/end-before`` tags.

To install on :ref:`supported platforms<supported platforms>`, please upgrade your pip:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m pip install --upgrade pip 

    .. code-tab:: powershell Windows x64
        
        PS > py -3 -m pip install --upgrade pip

.. note::
    
    Apple M1 users should be aware that due to numpy support limitations they will need to use Python
    >=3.8.


Now that your Python environment has an up-to-date pip, you can install ouster-sdk:


.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m pip install 'ouster-sdk[examples]'

    .. code-tab:: powershell Windows x64

        PS > py -3 -m pip install 'ouster-sdk[examples]'


.. note::
    
    While the optional examples component is not required for ouster-sdk, we recommend installing it
    so you can run the various examples and Ouster's ``simple-viz`` Visualizer, which will enable you to
    easily confirm a successful installation visually on recorded data, as outlined below.
 

To check that you've successfully installed the latest version of the Ouster Python SDK, run the
following command and make sure that the ``ouster-sdk`` package is included in the output:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m pip list

    .. code-tab:: powershell Windows x64

        PS > py -3 -m pip list


.. _venv instructions: https://packaging.python.org/guides/installing-using-pip-and-virtual-environments/#creating-a-virtual-environment

You should see something like:

.. parsed-literal::

        ouster-sdk                    \ |release|\


Sample Data Visualization
--------------------------

.. todo::
   Whole visualization pieces need to be revised and rethink in a light of new PointViz.

Let's check quickly that everything in the installation is in order by visualizing :doc:`sample data
<sample-data>` using the Ouster ``simple-viz`` Visualizer.


Next Steps
-----------


To get a feel for working with the Ouster Sensor Python API, click next to proceed to the
:doc:`python/quickstart`.

For shortcuts and controls for the Ouster ``simaple-viz`` Visualizer, or to immediately visualize
data directly from an Ouster sensor, see :doc:`python/visualizer`.


C++
========

.. todo::
   Write the C++ installation piece. Or the current link is enough?

Ouster C++ libraries need to be build and installed from sources. Please refer to
:doc:`/cpp/building`.


ROS
========

ROS drivers are supported and needs to be build from sources. Please refer to :doc:`/ros/index`