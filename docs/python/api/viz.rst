============================
Module :mod:`ouster.sdk.viz`
============================

.. contents::
   :local:
   :depth: 4

.. automodule:: ouster.sdk.viz

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

.. autoattribute:: spezia_palette
   :annotation: = spezia colors

.. autoattribute:: calref_palette
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