XYZLut & Destaggering
=====================

Once you have acquired a ``LidarFrame`` object, from a ``FrameSetSource`` as shown in the :doc:`consumption section <../consumption/using-the-api>`, you can perform numerous processing tasks.

As a reminder, the :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` class batches lidar
packets by full rotations into accessible fields of the appropriate type. 

The ``LidarFrame`` object holds data in 2D fields (like ``RANGE``, ``SIGNAL``, ``REFLECTIVITY``), and the SDK provides utilities to:

- Project 2D fields into 3D point clouds.
- Re-format 2D data for visualization and computer vision tasks.
- Filter, mask, and modify frame data.

Sample recordings are available from the datasets listed at :doc:`sample-data-download </getting-started/download_data>`.

Projecting to 3D (Point Clouds)
-------------------------------

The most common processing task is converting the 2D RANGE field into a 3D point cloud (XYZ points). The SDK performs this efficiently using a pre-computed Look-Up Table (LUT).

.. rubric:: **Common Setup**

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py
      
      .. literalinclude:: /../python/src/ouster/sdk/examples/core.py
         :language: python
         :start-after: [doc-stag-plot-xyz-imports]
         :end-before: [doc-etag-plot-xyz-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/python/representations.py>`__
         :dedent:
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/representations.cpp
         :language: cpp
         :start-after: [doc-stag-xyzlut-imports]
         :end-before: [doc-etag-xyzlut-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/cpp/representations.cpp>`__
         :dedent:


To convert 2D lidar data into a 3D point cloud,

- you first create a reusable lookup table (``XYZLut``) from your sensor's metadata. This table stores the 3D direction for every pixel. Then this table is applied to your 2D range image using the cartesian function, which efficiently calculates the final 3D (x, y, z) coordinates for every point.

The process involves two steps:

- **Build the lookup once**: run :ouster:class:`XYZLut <py=ouster.sdk.core.data.XYZLut|cpp=ouster::sdk::core::XYZLutT>` with your SensorInfo; it precomputes the per-pixel direction vectors.
- **Reuse it for every frame**: feed the LUT and a ``RANGE`` field into :ouster:func:`cartesian <cpp=ouster::sdk::core::cartesian>` (C++) or call the Python LUT object; it multiplies each range value by its direction vector and returns the XYZ coordinates (Eigen::Matrix in C++, (H, W, 3) array in Python).

For guidance on loading ``SensorInfo`` metadata when replaying a PCAP file, see the :ref:`get-sensor-info`.
Once you have the sensor info, you are ready to create the XYZ look up table:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py
      
      .. literalinclude:: /../python/src/ouster/sdk/examples/core.py
         :language: python
         :start-after: [doc-stag-plot-xyz-points]
         :end-before: [doc-etag-plot-xyz-points]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/core.py>`__
         :dedent:
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/representations_example.cpp
         :language: cpp
         :start-after: [doc-stag-cpp-xyz]
         :end-before: [doc-etag-cpp-xyz]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/core.py>`__
         :dedent:

Now, you have your x, y, and z values, you can plot them easily to get something like:

.. figure:: /images/lidar_frame_xyz_84.png
   :align: center

   Point cloud from OS1 sample data (frame 84). Points colored by ``SIGNAL`` value.

You can find the complete code by clicking on the github icon on the code snippet or
on our `GitHub repository <https://github.com/ouster-lidar/ouster-sdk>`_.

For a fast way to use a sensor to run :py:class:`~ouster.sdk.core.XYZLut` and visualize a similar image, refer to the pre-built Python example included in the SDK.  
Follow the instructions in the :ref:`Python LidarFrame examples<ex-python-lidarframe>` section.

Using Sensor Extrinsics in the LUT
++++++++++++++++++++++++++++++++++

``use_extrinsics=True`` tells :py:class:`~ouster.sdk.core.XYZLut` to apply ``sensor_info.sensor_to_body`` with ``sensor_info.lidar_to_sensor_transform``
and use that combined 4×4 transform when building the lookup table (i.e., the LUT outputs points in the extrinsics frame rather than the sensor frame).
The ``sensor_info.sensor_to_body`` matrix still comes straight from the sensor metadata and defaults to the identity matrix if the JSON omits it.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py
      
      .. literalinclude:: _snippets/python/representations.py
         :language: python
         :start-after: [doc-stag-transform-extrinsics]
         :end-before: [doc-etag-transform-extrinsics]
         :class: doc-snippet scroll-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/python/representations.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/representations.cpp
         :language: cpp
         :start-after: [doc-stag-cpp-xyzlut-extrinsics]
         :end-before: [doc-etag-cpp-xyzlut-extrinsics]
         :class: doc-snippet scroll-snippet
         :caption: `View on GitHub <|github-src|examples/representations_example.cpp>`__
         :dedent:


Applying External Transforms to the LUT
----------------------------------------

Users may find that they wish to apply an extra transform while projecting to Cartesian coordinates.
Such a transform, likely an extrinsics matrix of some sort, can be baked directly into
:ouster:class:`XYZLut <py=ouster.sdk.core.data.XYZLut|cpp=ouster::sdk::core::XYZLutT>` constructor, which internally uses the :ouster:func:`make_xyz_lut <cpp=ouster::sdk::core::make_xyz_lut>` function. 
This is more efficient than transforming every point cloud after projection.

In the following code, ``transform`` represents the external transformation matrix:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py
      
      .. literalinclude:: _snippets/python/representations.py
         :language: python
         :start-after: [doc-stag-repr-transform-metadata]
         :end-before: [doc-etag-repr-transform-metadata]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/python/representations.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/representations.cpp
         :language: cpp
         :start-after: [doc-stag-xyzlut-metadata-transform]
         :end-before: [doc-etag-xyzlut-metadata-transform]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|features/processing/_snippets/cpp/representations_example.cpp>`__
         :dedent:

Using the ``use_extrinsics=False`` returns the sensor-frame directions/offsets with no extra transform.
Hence, to apply external transform, we retrieve the `SensorInfo`, multiply in their custom 4×4 matrix, and then build the LUT—so the LUT bakes in the pose we supplied instead of applying it afterward.

**NOTE** The SDK also provides a  :py:class::`ouster.sdk.core.XYZLutFloat` in Python, which can be used when one needs to save memory or work in float32.


.. _ex-staggered-and-destaggered:

2D Data Layout: Staggered vs. Destaggered
-----------------------------------------

The default representation of ``LidarFrame`` stores data in **staggered** columns, meaning
that each column contains measurements taken at a single timestamp. As the lasers flashing at each
timestamp are arranged over several different azimuths, the resulting 2D image if directly
visualized is not a natural image.

Let's take a look at a typical **staggered** representation:

.. figure:: /images/lidar_frame_staggered.png
   :align: center

   LidarFrame ``RANGE`` field visualized with ``matplotlib.pyplot.imshow()`` and simple gray
   color mapping for better look.

To re-format the data into a natural 2D image representation, you must destagger it.
We *destagger* the relevant field of the ``LidarFrame`` with
``destagger`` (:cpp:func:`~ouster::destagger`, :py:func:`.core.destagger` function):

Destaggering re-orders the columns to correspond to their azimuth angle instead of timestamps.
This is essential for 2D visualization and many 2D computer vision algorithms.


The API accepts an optional ``inverse`` flag - set it to ``true`` when you need to
undo a previous destagger by reapplying the per-row pixel shifts and return data to the original staggered layout.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/representations.py
         :language: python
         :start-after: [doc-stag-destagger]
         :end-before: [doc-etag-destagger]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/python/representations.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/representations_example.cpp
         :language: cpp
         :start-after: [doc-stag-destagger]
         :end-before: [doc-etag-destagger]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/representations_example.cpp>`__
         :dedent:


After destaggering, the 2D image makes visual sense.

.. rubric:: Python: Plot destaggered reflectivity image

To generate staggered and destaggered images yourself, you can try the following sample code:

.. literalinclude:: _snippets/python/viz.py
    :language: python
    :start-after: [doc-stag-destagger-viz]
    :end-before: [doc-etag-destagger-viz]
    :class: doc-snippet
    :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/python/viz.py>`__
    :dedent:

The above code gives the scene below (see the long strip at the bottom). We've magnified two patches
for better visibility atop.

.. figure:: /images/lidar_frame_destaggered.png
    :align: center

    **destaggered** LidarFrame ``RANGE`` field

After *destaggering*, we can see the scene contains a man on a bicycle, a few cars, and many trees.
This image now makes visual sense, and we can easily use this data in common visual task pipelines.


Reshaping XYZ to 2D
+++++++++++++++++++

Users may find that they wish to access the ``x``, ``y``, and ``z`` coordinates of a single return
in a similar way. As the conversion to Cartesian coordinates returns an ``Eigen::Array`` ``n x 3``,
with ``n = w * h``, reshaping the resulting array is necessary. 

We can combine our knowledge in projecting into Cartesian coordinates and destaggering using the
following function:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/representations.py
         :language: python
         :start-after: [doc-stag-py-repr-x-image]
         :end-before: [doc-etag-py-repr-x-image]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/python/representations.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/representations_example.cpp
         :language: cpp
         :start-after: [docs-stag-x-image-form]
         :end-before: [docs-etag-x-image-form]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/representations_example.cpp>`__
         :dedent:

This demonstrates the functionality with ``x``, but it can be easily expanded to cover ``y`` and
``z`` as well.

Correlating 2D and 3D Data
---------------------------

The direct correlation between 2D and 3D representations in an Ouster sensor provides a powerful
framework for working with the data. 

Destaggering allows you to use 2D and 3D representations simultaneously. After destaggering, the pixel at (row, col) in a 2D field (like RANGE) corresponds to the 3D point at (row, col) in a destaggered XYZ point cloud.

This allows you to perform 2D operations (like image-based filtering) and apply the results directly to your 3D data.

The example below filters a point cloud based on both RANGE (distance) and azimuth angle (a 2D column index).

.. rubric:: Setup

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/core.py
         :start-after: [doc-stag-filter-3d-setup]
         :end-before: [doc-etag-filter-3d-setup]
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/core.py>`__
         :class: doc-snippet
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/representations.cpp
         :language: cpp
         :start-after: [doc-stag-filter-3d-setup]
         :end-before: [doc-etag-filter-3d-setup]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/cpp/representations.cpp>`__
         :dedent:

.. rubric:: 3D Destagger

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/core.py
         :start-after: [doc-stag-filter-3d-destagger]
         :end-before: [doc-etag-filter-3d-destagger]
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/core.py>`__
         :class: doc-snippet
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/representations.cpp
         :language: cpp
         :start-after: [doc-stag-filter-3d-destagger]
         :end-before: [doc-etag-filter-3d-destagger]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/cpp/representations.cpp>`__
         :dedent:

.. rubric:: Mask

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/core.py
         :start-after: [doc-stag-filter-3d-mask]
         :end-before: [doc-etag-filter-3d-mask]
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/core.py>`__
         :class: doc-snippet
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/representations.cpp
         :language: cpp
         :start-after: [doc-stag-filter-3d-mask]
         :end-before: [doc-etag-filter-3d-mask]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/cpp/representations.cpp>`__
         :dedent:

.. rubric:: Filter

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/core.py
         :start-after: [doc-stag-filter-3d]
         :end-before: [doc-etag-filter-3d]
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/core.py>`__
         :class: doc-snippet
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/representations.cpp
         :language: cpp
         :start-after: [doc-stag-filter-3d]
         :end-before: [doc-etag-filter-3d]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/cpp/representations.cpp>`__
         :dedent:   

Since we’d like to filter on azimuth angles, first we first destagger both the 2D and 3D points, so that our columns in the HxW representation correspond to azimuth angle, not timestamp.

Then we filter the 3D points xyz_destaggered by comparing the range measurement to range_min, which we can do because there is a 1:1 correspondence between the columns and rows of the destaggered representations of xyz_destaggered and range_destaggered. (Similarly, there would be a 1:1 correspondence between the staggered representations xyz and range, where the columns correspond with timestamp).

Finally, we select only the azimuth columns we’re interested in. In this case, we’ve arbitrarily chosen ``azimuth_fraction = 0.75``, which keeps the first 75% of the frame (~270° of rotation).

For a fast way to use a sensor to run the above examples and visualize the output, refer to the pre-built Python example included in the SDK.  
Follow the instructions in the :ref:`Python LidarFrame examples<ex-python-lidarframe>` section.



.. rubric:: Full code

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/core.py
         :start-after: [doc-stag-filter-3d-full]
         :end-before: [doc-etag-filter-3d-full]
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/core.py>`__
         :class: doc-snippet scroll-snippet
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/representations.cpp
         :language: cpp
         :start-after: [doc-stag-filter-3d-full]
         :end-before: [doc-etag-filter-3d-full]
         :class: doc-snippet scroll-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/cpp/representations.cpp>`__
         :dedent:   

Populating LidarFrames
---------------------

This reference has covered how to access, project, and destagger ``LidarFrame``
contents. But in order for ``LidarFrames`` to be useful, we need a way to populate them with packet
data! 

For convenience, the Ouster Python SDK provides batching helpers for both live and recorded data.

See :doc:`the data consumption section <../consumption/lidar_frame>` for more information.


Adding custom fields to a LidarFrame
-----------------------------------

It's possible to add custom fields to a ``LidarFrame``. This is especially useful if you
want to add custom data to an OSF file for visualization purposes. 

You can find more information about this in the documentation :ref:`custom-fields`.

