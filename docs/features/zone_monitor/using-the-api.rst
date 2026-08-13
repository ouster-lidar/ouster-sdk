Using the API
=============

Creating zones
==============

The LiDAR can monitor zones that are unobstructed and within its field of view. Conversely, a configuration that specifies a zone outside the field of view is invalid. Additionally, if a zone is behind an obstruction or occlusion, the LiDAR will not be able to determine whether an object occupies it. However, the LiDAR will report the number of points that occlude a zone, which helps indicate whether the sensor can monitor the zone effectively.

Users can specify zones either using 3D meshes that the sensor renders to an internal image-based representation, or directly as range image pairs (bypassing the rendering step.) The following sections describe both approaches, as well as how to specify the zone's criteria for triggering.

.. tip:: The SDK currently supports loading meshes from STL files. A large number of tools, such as `Blender <https://blender.org>`_ can produce STL files from 3D models.

Defining a zone from a 3D mesh
------------------------------

When the user uploads a 3D mesh (e.g., in the form of an STL file) as part of a zone's configuration, the sensor renders this geometry into a pair of near/far distance images that it uses during each frame to determine which LiDAR measurements are within the zone. This rendering process checks the intersections between each LiDAR beam and the 3D mesh. Because of this, the LiDAR also requires that each zone's geometry meets some basic criteria.

.. tip:: Zone Monitor assumes meter units for mesh vertex coordinate values. Although the STL format does not support a scale factor, many 3D modeling tools allow for one when exporting a model to STL. When exporting STL files for Zone Monitor zones, ensure that the scale factor is set correctly so that the geometry is the correct size in meters.

First, each of the LiDAR's beams should intersect the 3D geometry no more than two times. If a beam intersects the geometry two or more times, the LiDAR considers the portion between the closest and furthest intersections to be inside the zone and all other portions outside. If a beam intersects the geometry exactly once, then the LiDAR considers the area between the sensor and the intersection to be inside the zone and all other portions outside. Importantly, this means that the LiDAR cannot monitor all 3D geometries. The Visualization section later in this document describes how to visualize any potential difference between the specified geometry and what the sensor can monitor.

.. figure:: /images/raycasting.svg
    :align: center
    :alt: A top-down view of some example mesh shapes and how they affect rendering results
    :width: 100%

    A top-down view of some example mesh shapes and how they affect rendering results

Another important criterion for a valid zone configuration is that the number of points required for the sensor to trigger the zone must not be greater than the number of beams that intersect it. (If this criterion is not met, the LiDAR never triggers the zone because the number of LiDAR measurements would always be fewer than the number of points the configuration specifies for triggering.)

Defining a zone from a pair of depth images directly using ZRB format
---------------------------------------------------------------------

Users can also specify a pair of images representing the near and far ranges of the zone directly (rather than having the sensor render them from a 3D mesh.) The SDK provides an API for doing this, including saving the results to a ZRB file - a proprietary format for encoding a zone as a near/far image pair. Using this approach is subject to some requirements and limitations:

* The near/far images are 32-bit unsigned integer images representing distance in millimeters.
* The near/far images must be in the LiDAR's coordinate frame.
* The near/far images for a zone must have the same resolution.
* The near value for a pixel must always be less than or equal to the far value for the corresponding pixel.
* The resolution must be the same for all images within a given zone set.

Specifying zone triggering criteria
-----------------------------------

Zones have several parameters that determine how and when they trigger. The most important of these is the number of points that must be present within the zone for it to trigger. This parameter is an integer. Other parameters include the frame count, which specifies the number of consecutive LiDAR frames that must meet the triggering criteria before the zone triggers, and the mode, which specifies whether the zone triggers when the number of points within it is greater than or less than the specified number of points. Here is a complete list of zone parameters:

* ``mode`` - either ``OCCUPANCY`` or ``VACANCY``.
* ``point_count`` - when the mode is ``OCCUPANCY`` there must be this many points or more within the zone for it to trigger; when the mode is ``VACANCY`` there must be fewer than this many points within the zone for it to trigger.
* ``frame_count`` - the number of consecutive LiDAR frames that must meet the triggering criteria before the zone triggers.
* ``label`` - an optional string label for the zone, useful for identifying zones in application code.

Refer to the Python :ouster:class:`Zone <py=ouster.sdk.core.Zone|cpp=ouster::sdk::core::Zone>` API reference for more information about these parameters.

So far, we've described how to define a single zone's geometry and triggering criteria. The following sections will explain how to programmatically create a complete set of zones from either mesh-derived or image-derived zone geometry and upload it to the sensor.

Creating a ``ZoneSet``
======================

The Python class :ouster:class:`ZoneSet <py=ouster.sdk.core.ZoneSet|cpp=ouster::sdk::core::ZoneSet>` are for storing a complete Zone Monitor configuration consisting of multiple zones, their geometry, and their associated metadata.

Specifying a sensor-to-body transform for the ZoneSet
------------------------------------------------------

The ``ZoneSet`` requires a sensor-to-body transform that specifies the pose of the sensor relative to the zones. Using this transform allows creating zone geometry that is defined in a body frame (e.g., the vehicle frame) rather than the sensor frame. This is useful because the zones often correspond to physical features of the body (e.g., the sides of a vehicle) rather than features of the sensor itself. Using an identity matrix for the transform means that the zones' geometries use the sensor's coordinate frame.

The `Ouster Sensor Docs`_ contain a detailed description of the sensor's coordinate frames.

.. _Ouster Sensor Docs: https://docs.ouster.com/sensor-docs/coordinate-system#sensor-coordinate-frame

.. important:: The zone geometry must remain fully or partially within the sensor's FOV after applying the sensor-to-body transform; otherwise, the zone set is invalid.

.. _creating-a-zoneset-using-the-sdk:

Creating a ``ZoneSet`` using the SDK
------------------------------------

The following examples describe how to create and define zone sets that the LiDAR can render from STLs or directly from ZRBs, respectively.

In both cases, the SDK writes the ``ZoneSet`` as a zip file, which the sensor accepts as an upload either via the HTTP API or from the web UI.

Start by importing the necessary modules:

.. rubric:: Imports

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/zone_monitor_example.py
         :language: python
         :start-after: [doc-stag-stl-imports]
         :end-before: [doc-etag-stl-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/zone_monitor_example.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/zone_monitor_zone_set.cpp
         :language: cpp
         :start-after: [doc-stag-stl-imports]
         :end-before: [doc-etag-stl-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/zone_monitor_zone_set.cpp>`__
         :dedent:

The following examples show how to create a zone set from STL meshes or directly from ZRB range images.

Creating an STL zone set
------------------------

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/zone_monitor_example.py
         :language: python
         :start-after: [doc-stag-stl-zone-set]
         :end-before: [doc-etag-stl-zone-set]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/zone_monitor_example.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/zone_monitor_zone_set.cpp
         :language: cpp
         :start-after: [doc-stag-stl-zone-set]
         :end-before: [doc-etag-stl-zone-set]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/zone_monitor_zone_set.cpp>`__
         :dedent:

Creating a ZRB zone set
-----------------------

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/zone_monitor_example.py
         :language: python
         :start-after: [doc-stag-zrb-zone-set]
         :end-before: [doc-etag-zrb-zone-set]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/zone_monitor_example.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/zone_monitor_zone_set.cpp
         :language: cpp
         :start-after: [doc-stag-zrb-zone-set]
         :end-before: [doc-etag-zrb-zone-set]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/zone_monitor_zone_set.cpp>`__
         :dedent:

Configuring the LiDAR
=====================

Enabling Zone Monitor on an Ouster LiDAR requires uploading and applying a valid Zone Monitor configuration and specifying a destination for Zone Monitor UDP packets. Users can do both using the SDK.

.. important:: When uploading a zone set to a sensor all of its zones must either be STL-based or ZRB-based; the sensor does not support mixing both types in a single zone set.

Uploading and applying the zone set to the sensor
-------------------------------------------------

Once the zone set zip file has been created, use the SDK to upload and apply it to the sensor.

.. rubric:: Imports

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/zone_monitor_example.py
         :language: python
         :start-after: [doc-stag-upload-imports]
         :end-before: [doc-etag-upload-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/zone_monitor_example.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/zone_monitor_zone_set.cpp
         :language: cpp
         :start-after: [doc-stag-stl-imports]
         :end-before: [doc-etag-stl-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/zone_monitor_zone_set.cpp>`__
         :dedent:

With the imports in place, upload the zone set and apply it to the sensor:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/zone_monitor_example.py
         :language: python
         :start-after: [doc-stag-upload-zone-set]
         :end-before: [doc-etag-upload-zone-set]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/zone_monitor_example.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/zone_monitor_zone_set.cpp
         :language: cpp
         :start-after: [doc-stag-upload-zone-set]
         :end-before: [doc-etag-upload-zone-set]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/zone_monitor_zone_set.cpp>`__
         :dedent:

Specifying a destination for zone packets
-----------------------------------------

By default, :ouster:func:`open_source <py=ouster.sdk.open_source|cpp=ouster::sdk::open_source>` function automatically configures the LiDAR to send Zone Monitor packets to the same IP address and port as the main LiDAR data packets.

If users wish to specify a different destination, they can do so using the SDK as follows.

.. rubric:: Imports

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/zone_monitor_zone_states_example.py
         :language: python
         :start-after: [doc-stag-zone-state-imports]
         :end-before: [doc-etag-zone-state-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/zone_monitor_zone_states_example.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/zone_monitor_zone_states.cpp
         :language: cpp
         :start-after: [doc-stag-zone-state-imports]
         :end-before: [doc-etag-zone-state-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/zone_monitor_zone_states.cpp>`__
         :dedent:

Once the imports are updated, specify a destination for zone packets:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/zone_monitor_zone_states_example.py
         :language: python
         :start-after: [doc-stag-set-zm-udp-dest]
         :end-before: [doc-etag-set-zm-udp-dest]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/zone_monitor_zone_states_example.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/zone_monitor_zone_states.cpp
         :language: cpp
         :start-after: [doc-stag-set-zm-udp-dest]
         :end-before: [doc-etag-set-zm-udp-dest]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/zone_monitor_zone_states.cpp>`__
         :dedent:


Specifying the live zone ids
----------------------------

Finally, users must specify one or more live zones. The LiDAR only monitors up to 16 live zones at a time, but users can change between different sets of live zones quickly using the SDK.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/zone_monitor_zone_states_example.py
         :language: python
         :start-after: [doc-stag-set-live-zones]
         :end-before: [doc-etag-set-live-zones]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/zone_monitor_zone_states_example.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/zone_monitor_zone_states.cpp
         :language: cpp
         :start-after: [doc-stag-set-live-zones]
         :end-before: [doc-etag-set-live-zones]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/zone_monitor_zone_states.cpp>`__
         :dedent:

Reading Zone Monitor output
===========================

An Ouster LiDAR with Zone Monitor enabled emits zone packets that the SDK automatically batches into ``LidarFrame`` objects yielded from ``FrameSetSource``. This means determining whether an object has entered, exited, or is obstructing a zone is easy. The ``ZONE_STATES`` field in the ``LidarFrame`` contains a `numpy structured array`_ containing the state of all live zones. The ``ZONE_OCCUPANCY`` field is an image, the same dimensions as the other pixel fields, where each pixel is a bitset indicating which zones overlap with the corresponding lidar point.

Refer to the :ouster:class:`ZoneState <py=ouster.sdk.core.ZoneState|cpp=ouster::sdk::core::ZoneState>` API reference for more information about these parameters.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/zone_monitor_zone_states_example.py
         :language: python
         :start-after: [doc-stag-read-zone-states]
         :end-before: [doc-etag-read-zone-states]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/zone_monitor_zone_states_example.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/zone_monitor_zone_states.cpp
         :language: cpp
         :start-after: [doc-stag-read-zone-states]
         :end-before: [doc-etag-read-zone-states]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/zone_monitor_zone_states.cpp>`__
         :dedent:


Emulation
=========

The SDK's Zone Monitor Emulation feature enables testing Zone Monitor configurations with or without a sensor.

.. note:: The Ouster SDK C++ API does not yet support Zone Monitor Emulation.

The following is an example of how to use the emulation feature in Python.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/zone_monitor_emulation_example.py
         :language: python
         :start-after: [doc-stag-zone-monitor-emulation]
         :end-before: [doc-etag-zone-monitor-emulation]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/zone_monitor_emulation_example.py>`__
         :dedent:


For CLI-based emulation, see :doc:`using-cli`.

.. _numpy structured array: https://numpy.org/doc/stable/user/basics.rec.html
