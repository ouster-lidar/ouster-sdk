============
lidar_scan.h
============

.. contents::
    :local:

LidarScan
=========

.. doxygenclass:: ouster::LidarScan
    :members:

.. doxygengroup:: ouster_client_lidar_scan_cartesian
    :content-only:

.. doxygengroup:: ouster_client_lidar_scan_operators
    :content-only:

Destagger
=========

.. doxygengroup:: ouster_client_destagger
    :content-only:

XYZLut
======

.. doxygenstruct:: ouster::XYZLut
    :members:

.. doxygenfunction:: ouster::make_xyz_lut(size_t w, size_t h, double range_unit, const mat4d& beam_to_lidar_transform, const mat4d& transform, const std::vector<double>& azimuth_angles_deg, const std::vector<double>& altitude_angles_deg)

.. doxygenfunction:: ouster::make_xyz_lut(const sensor::sensor_info& sensor)

ScanBatcher
===========

.. doxygenclass:: ouster::ScanBatcher
    :members:

