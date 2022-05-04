============================
Module :mod:`ouster.sdk.viz`
============================

.. contents::
   :local:
   :depth: 4

.. py:currentmodule:: ouster.sdk.viz

Ouster sensor data visualization tools. Implemented in C++ OpenGL and wrapped with Python bindings.

For additional information please refer to :doc:`/python/viz/viz-api-tutorial`.

----

Core
====

.. autoclass:: PointViz
   :members:

.. autofunction:: add_default_controls

.. autoclass:: LidarScanViz
   :members:

.. autoclass:: SimpleViz
   :members:

.. autoattribute:: ouster.sdk.viz.spezia_palette
   :annotation: = spezia colors

.. autoattribute:: ouster.sdk.viz.calref_palette
   :annotation: = calref colors


Visual Primitives
=================

``Cloud`` for 3D Point Cloud
----------------------------

.. autoclass:: Cloud
   :members:

``Image`` for 2D image
-----------------------

.. autoclass:: Image
   :members:

``Cuboid`` for 3D enclosure
----------------------------

.. autoclass:: Cuboid
   :members:

``Label`` for 2D and 3D text
----------------------------

.. autoclass:: Label
   :members:


``Camera`` and ``TargetDisplay``
=================================

.. autoclass:: Camera
   :members:

.. autoclass:: TargetDisplay
   :members:

Events ``WindowCtx``
====================

.. autoclass:: WindowCtx
   :members: