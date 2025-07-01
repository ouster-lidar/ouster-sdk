downsample.h
============

.. contents::
   :local:

Functions
---------

.. doxygenfunction:: ouster::core::voxel_downsample(const Eigen::Matrix< typename DerivedPts::Scalar, 3, 1 > &voxel_size3, const DerivedPts &pts, const DerivedAttrib &attribs, DerivedOutPts &out_pts, DerivedOutAttrib &out_attribs, int min_pts_per_voxel)
   :project: cpp_api

.. doxygenfunction:: ouster::core::voxel_downsample(typename DerivedPts::Scalar voxel_size, const DerivedPts &pts, const DerivedAttrib &attribs, DerivedOutPts &out_pts, DerivedOutAttrib &out_attribs, int min_pts_per_voxel)
   :project: cpp_api