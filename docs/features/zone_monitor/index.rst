.. _zone_monitor.rst:

============
Zone Monitor
============

Introduction
============

Ouster Firmware 3.2 introduces Zone Monitor, which enables an Ouster LiDAR sensor to monitor multiple user-provided 3D zones within its field of view for the presence or absence of objects. The Ouster LiDAR monitors these zones without the need for extra equipment or software, making this feature useful for a wide variety of security and robotics applications, such as perimeter intrusion detection and collision avoidance.

Ouster SDK 0.16.0 provides full support for this feature, allowing users to easily read and write Zone Monitor configuration from and to Ouster LiDAR sensors, record and visualize the zones and their states over time, and test a Zone Monitor configuration using pre-recorded data or data recorded from a sensor running older firmware.

This document describes how the feature works, its use cases and capabilities, how to configure an Ouster LiDAR to use this feature, and how to use the SDK to create, visualize, and test Zone Monitor configurations. The `Ouster Firmware documentation <https://ouster.com/sensor-docs/fw-user-manual/zone-monitor>`__ contains further information about Zone Monitor, including relevant extensions to the LiDAR's UDP protocol and HTTP APIs.

.. caution::
    FW 3.2 and Ouster Rev 7 and earlier sensor models do not yet support using Zone Monitor for use in safety-critical applications.

How it works
============

An Ouster LiDAR supports configurations of as many as 128 zones, with support for live monitoring of up to 16 zones at a time.

The configuration for each zone consists of a 3D geometry and triggering criteria. When the user uploads and applies a Zone Monitor configuration to the LiDAR, the LiDAR uses the zone geometry to render a pair of near/far distance images that it uses to determine which LiDAR measurements the zone contains during each LiDAR frame. During each frame, the LiDAR's firmware also checks the triggering criteria for each zone and transmits a packet containing information for each live zone, including whether that zone has met the triggering criteria (i.e. an object has entered or left the zone) or whether there is an obstruction preventing the LiDAR from determining the status of the zone.

Additionally, Ouster Firmware 3.2 introduces new LiDAR packet formats that include per-pixel zone occupancy information. When one of these formats is in use, each LiDAR point contains an additional bitset that indicates which live zones contain the corresponding point.

Use cases and capabilities
==========================

Zone Monitor is a versatile feature that can be used in a variety of applications. Some example use cases include the following:

* Perimeter intrusion detection
* Collision avoidance
* Cliff detection

Example use case: a vehicle-based LCAS system
----------------------------------------------

Zone Monitor is a useful feature for many robotics applications, such as collision avoidance. This section provides a simplified description of how to integrate Zone Monitor into a vehicle-based LiDAR Collision Avoidance (LCAS) system.

In this system, a LiDAR is mounted on the front of a vehicle. The system must stop the vehicle if an object is present in the vehicle's predicted drive path, which the controller computes from operator inputs.

The system implementer specifies several zones. An LCAS controller then determines which subset of zones should be live based on the vehicle's current speed and direction of travel. The vehicle's speed and direction determine its move plan, which in turn determines which zones to monitor. For example, if the operator turns the steering wheel to the right, the LCAS controller recomputes the predicted vehicle path and tells the LiDAR to activate the zones on the right side of the vehicle. If the operator moves the steering wheel back to the center, the LCAS controller tells the LiDAR to monitor the zones directly in front of the vehicle.

.. figure:: /images/lcas_zones.svg
    :align: center
    :alt: A diagram depicting an example LCAS zone configuration, with live zones depending on steering angle
    :width: 100%

    A diagram depicting an example LCAS zone configuration, with live zones depending on steering angle

When the sensor sets a zone's state to *triggered*, it sends a packet back to the controller, which then alerts the operator to slow or stop.

.. figure:: /images/zone_monitor_lcas_case_study.svg
    :align: center
    :alt: A block diagram depicting components in an LCAS system for a vehicle
    :width: 100%

    A diagram depicting components in an LCAS system for a vehicle

Getting started
===============

See :doc:`using-the-api` for a full walkthrough of creating, rendering, and uploading zone configurations. If you already have a zone set config zip and a data source, :doc:`using-cli` shows how to quickly emulate and visualize zones from the command line.

.. toctree::
   :hidden:
   :maxdepth: 1

   using-cli
   using-the-api