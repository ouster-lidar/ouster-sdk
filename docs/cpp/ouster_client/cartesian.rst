cartesian.h
===========

.. contents::
   :local:
   
Typedefs
--------

.. doxygentypedef:: ouster::PointsT
   :project: cpp_api

.. doxygentypedef:: ouster::PointsD
   :project: cpp_api

.. doxygentypedef:: ouster::PointsF
   :project: cpp_api

Functions
---------

.. doxygenfunction:: ouster::cartesianT(PointsT< T > &points, const Eigen::Ref< const img_t< uint32_t > > &range, const PointsT< T > &direction, const PointsT< T > &offset)
   :project: cpp_api

.. doxygenfunction:: ouster::cartesianT(const Eigen::Ref< const img_t< uint32_t > > &range, const PointsT< T > &direction, const PointsT< T > &offset)
   :project: cpp_api

.. doxygenfunction:: ouster::cartesianT(const LidarScan &scan, const PointsT< T > &direction, const PointsT< T > &offset)
   :project: cpp_api
