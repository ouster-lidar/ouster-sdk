===============================
PointViz Tutorial & API Usage
===============================

.. contents::
   :local:


Environment and General Setup
=============================

In this interactive tutorial we explore the Python bindings of the :class:`.viz.PointViz` and how we
can use it programmatically to visualize various 2D and 3D scenes.

.. note::

    We assume you have downloaded the :doc:`sample data </sample-data>` and set
    ``SAMPLE_DATA_PCAP_PATH`` and ``SAMPLE_DATA_JSON_PATH`` to the locations of the OS1 128 sample
    data pcap and json files correspondingly.

Before proceeding, be sure to install ``ouster-sdk`` Python package, see :ref:`OusterSDK Python
Installation<installation-python>` for details.

This interactive tutorial will open a series of visualizer windows on your screen. Each example can
be exited by pressing ``ESC`` or the exit button on the window; doing so will open the next
visualizer window. 

Let's start the tutorial::

    python3 -m ouster.sdk.examples.viz $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH

Now you can proceed through each example, labeled by numbers to match your screen output.

Creating an empty ``PointViz`` window
=====================================

:class:`.viz.PointViz` is the main entry point to the Ouster visualizer, and it keeps track of the
window state, runs the main visualizations loop, and handles mouse and keyboard events.

To create the PointViz window (and see a black screen as nothing has been added), you can try the
following:

.. literalinclude:: /../python/src/ouster/sdk/examples/viz.py
    :start-after: [doc-stag-empty-pointviz]
    :end-before: [doc-etag-empty-pointviz]
    :emphasize-lines:  2
    :linenos:
    :dedent:

As expected we see a black empty window:

.. figure:: /images/viz-tutorial/empty_point_viz.png
    :align: center

    Empty ``PointViz`` window

Now let's try to add some visual objects to the viz. Hit ``ESC`` or click the exit button on the
window to move to the next example.


Images and Labels
=================

Some of the basic 2D objects we can add include the 2D :class:`.viz.Label` and the
:class:`.viz.Image`. To add these, we need an understanding of the 2D Coordinate System for :class:`.viz.PointViz`.

The ``Image`` object
--------------------

To create a 2D Image you need to use the :class:`.viz.Image` object. Currently only normalized
grayscale images are supported, though you can use :meth:`.viz.Image.set_mask`, which accepts RGBA
masks, to get color.

The :class:`.viz.Image` screen coordinate system is *height-normalized* and goes from bottom to top
(``[-1, +1]``) for the ``y`` cordinate, and from left to right (``[-aspect, +aspect]``) for the
``x`` coordidate, where::
    
    aspect = viewport width in Pixels / viewport height in Pixels

Here is how we create a square image and place it in the center:

.. literalinclude:: /../python/src/ouster/sdk/examples/viz.py
    :start-after: [doc-stag-image-pos-center]
    :end-before:  [doc-etag-image-pos-center]
    :emphasize-lines:  2-3
    :linenos:
    :dedent:

Expected result: (without top and bottom labels)

.. figure:: /images/viz-tutorial/image_set_pos_center.png
    :align: center

    :meth:`.viz.Image.set_position` to center image

Window-aligned Images
---------------------

To align an image to the left or right screen edge, use :meth:`.viz.Image.set_hshift`, which accepts
a *width-normalized* (``[-1, 1]``) horizontal shift which is applied to the image position after the
:meth:`.viz.Image.set_position` values are first applied.

Here is how we place an image to the left edge of the screen:

.. literalinclude:: /../python/src/ouster/sdk/examples/viz.py
    :start-after: [doc-stag-image-pos-left]
    :end-before:  [doc-etag-image-pos-left]
    :linenos:
    :dedent:

Expected result:

.. figure:: /images/viz-tutorial/image_set_pos_left.png
    :align: center

    :meth:`.viz.Image.set_position` with :meth:`.viz.Image.set_hshift` to align image to the left

And here's how we place an image to the right edge of the screen:

.. literalinclude:: /../python/src/ouster/sdk/examples/viz.py
    :start-after: [doc-stag-image-pos-right]
    :end-before:  [doc-etag-image-pos-right]
    :linenos:
    :dedent:

Expected result:

.. figure:: /images/viz-tutorial/image_set_pos_right.png
    :align: center

    :func:`.viz.Image.set_position` to align image to the right

Finally, here's how we place an image to the right edge and bottom of the screen:

.. literalinclude:: /../python/src/ouster/sdk/examples/viz.py
    :start-after: [doc-stag-image-pos-right-bottom]
    :end-before:  [doc-etag-image-pos-right-bottom]
    :linenos:
    :dedent:

Expected result:

.. figure:: /images/viz-tutorial/image_set_pos_bot_right.png
    :align: center

    :func:`.viz.Image.set_position` to align image to the bottom and right

Cool! Let's move on to more sophisticated images!


LidarScan Fields as Images
--------------------------

Now we can use :class:`.viz.Image` to visualize ``LidarScan`` fields data as images. In this example
we show the ``RANGE`` and ``SIGNAL`` fields images positioned on the top and bottom of the screen
correspondingly.

.. note::

    If you aren't familiar with the ``LidarScan``, please see :doc:`/reference/lidar-scan`. A single
    scan contains a full frame of lidar data.

So how do we do it? 

.. literalinclude:: /../python/src/ouster/sdk/examples/viz.py
    :start-after: [doc-stag-scan-fields-images]
    :end-before:  [doc-etag-scan-fields-images]
    :emphasize-lines:  17-34
    :linenos:
    :dedent:

In the highlighted lines, you can see that we're simply using :func:`.viz.Image()`,
:func:`.viz.set_image()`, and :func:`.viz.set_position()` like we did before, but this time with
more interesting data! 

Expected result:

.. figure:: /images/viz-tutorial/lidar_scan_fields_images.png
    :align: center

    ``LidarScan`` fields ``RANGE`` and ``SIGNAL`` visualized with :class:`.viz.Image`


2D ``Label`` object
--------------------

2D labels are represented with :class:`.viz.Label` object and use a slightly different coordinate
system that goes from left to right ``[0, 1]`` for the ``x`` coordinate, and from top to bottom
``[0, 1]`` the for ``y`` coordinate.

Let's apply the labels to our already visualized ``LidarScan`` field images:

.. literalinclude:: /../python/src/ouster/sdk/examples/viz.py
    :start-after: [doc-stag-scan-fields-images-labels]
    :end-before:  [doc-etag-scan-fields-images-labels]
    :linenos:
    :dedent:

Expected result:

.. figure:: /images/viz-tutorial/lidar_scan_fields_images_labels.png
    :align: center

    ``LidarScan`` fields images captioned with 2D :class:`.viz.Label`

.. note::

    Currently the :class:`.viz.Image` has a different 2D coordinate system than the
    :class:`.viz.Label`. This is likely to change in the future.


Point Clouds: the ``Cloud`` object
==================================

Point Cloud visualization implemented via the :class:`.viz.Cloud` object can be used in two ways:

    - :ref:`Structured Point Clouds <structured-point-cloud>`, where 3D points are defined with 2D
      field images (i.e. ``LidarScan`` fields images) 
    - :ref:`Unstructured Point Clouds <unstructured-point-cloud>`, where 3D points are defined
      directly as a set of XYZ 3D points

Let's take a closer look at each of these.

.. _structured-point-cloud:

Structured Point Cloud
----------------------

The Ouster sensor produces a structured point cloud as a 2D range image which can be projected into
3D Cartesian coordinates with a pre-generated lookup table. For this reason the internal
implementation of :class:`.viz.Cloud` applies the lookup table transform automatically generated
from :class:`.client.SensorInfo` (metadata object) and 2D ``RANGE`` image as an input.

.. note::

   Please refer to :doc:`/reference/lidar-scan` for details about interal ``LidarScan``
   representations and its basic operations.


To visualize :class:`.client.LidarScan` object we use the following code:

.. literalinclude:: /../python/src/ouster/sdk/examples/viz.py
    :start-after: [doc-stag-scan-structured]
    :end-before:  [doc-etag-scan-structured]
    :linenos:
    :dedent:

Expected result:

.. figure:: /images/viz-tutorial/lidar_scan_structured.png
    :align: center

    ``LidarScan`` Point Cloud with :class:`.viz.Cloud` as structured


.. _unstructured-point-cloud:

Unstructured Point Clouds
-------------------------

Point Clouds that are represented as a set of XYZ 3D points are called *unstructured* for our
purposes.

It's possible to set the :class:`.viz.Cloud` object by setting the 3D points directly, which is
useful when you have unstructured point clouds.

.. literalinclude:: /../python/src/ouster/sdk/examples/viz.py
    :start-after: [doc-stag-scan-unstructured]
    :end-before:  [doc-etag-scan-unstructured]
    :emphasize-lines:  5-6
    :linenos:
    :dedent:

You should see the same visualization as for the structured point clouds.


Example: 3D Axes Helper as Unstructured Points
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :class:`.viz.Cloud` object also supports color RGBA masks. Below is an example how we can use
unstructured point clouds to draw 3D axes at the origin:

.. literalinclude:: /../python/src/ouster/sdk/examples/viz.py
    :start-after: [doc-stag-axes-helper]
    :end-before:  [doc-etag-axes-helper]
    :emphasize-lines:  18-23
    :linenos:
    :dedent:

Expected result:

.. figure:: /images/viz-tutorial/axes_helper_unstructured.png
    :align: center

    3D Axes Helper with :class:`.viz.Cloud` unstructured and color RGBA masks


``LidarScanViz`` for point cloud with fields images
---------------------------------------------------

To make it even more easier to explore the data of :class:`.client.LidarScan` objects, we provide a
higher-order visual component :class:`.viz.LidarScanViz` that enables:

- 3D point cloud and two 2D fields images in one view
- color palettes for 3D point cloud coloration
- point cloud point size increase/decrease
- toggles for view of different fields images and increasing/decreaseing their size
- dual return point clouds and fields images support
- key handlers for all of the above

The majority of keyboard operations that you can see in :ref:`simple-viz keymaps
<simple-viz-keymap>` implemented by :class:`.viz.LidarScanViz`.

Let's look at how we can explore a :class:`.client.LidarScan` object with a couple of lines:

.. literalinclude:: /../python/src/ouster/sdk/examples/viz.py
    :start-after: [doc-stag-lidar-scan-viz]
    :end-before:  [doc-etag-lidar-scan-viz]
    :emphasize-lines:  2,5,8
    :linenos:
    :dedent:

Not a lot of lines of code needed to visualize all the information desired!

Expected result:

.. figure:: /images/viz-tutorial/lidar_scan_viz.png
    :align: center

    ``LidarScan`` Point Cloud with :class:`.viz.LidarScanViz`


3D ``Label`` object
--------------------

The same :class:`.viz.Label` object that we used for 2D labels can be used for putting text labels
in 3D space.

Here's an example of using 3D Labels:

.. literalinclude:: /../python/src/ouster/sdk/examples/viz.py
    :start-after: [doc-stag-lidar-scan-viz-labels]
    :end-before:  [doc-etag-lidar-scan-viz-labels]
    :linenos:
    :dedent:

Expected result:

.. figure:: /images/viz-tutorial/lidar_scan_viz_labels.png
    :align: center

    3D :class:`.viz.Label` superimposed onto ``LidarScanViz``


Example: Overlay 2D Images and 2D Labels
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Here's another example to show how one could add 2D Images, RGBA masks and 2D Labels added to :class:`.viz.LidarScanViz`:

.. literalinclude:: /../python/src/ouster/sdk/examples/viz.py
    :start-after: [doc-stag-overlay-images-labels]
    :end-before:  [doc-etag-overlay-images-labels]
    :linenos:
    :dedent:

Expected result:

.. figure:: /images/viz-tutorial/lidar_scan_viz_checkers.png
    :align: center

    2D :class:`.viz.Label` and :class:`.viz.Image` superimposed onto ``LidarScanViz``


Event Handlers
===============

Custom keyboard handlers can be added to handle key presses in :class:`.viz.PointViz`.

Keyboard handlers
------------------

We haven't yet covered the :class:`.viz.Camera` object and ways to control it. So here's a quick
example of how we can map ``R`` key to move the camera closer or farther from the target.

Random :meth:`.viz.Camera.dolly` change on keypress:

.. literalinclude:: /../python/src/ouster/sdk/examples/viz.py
    :start-after: [doc-stag-key-handlers]
    :end-before:  [doc-etag-key-handlers]
    :emphasize-lines:  1,9
    :linenos:
    :dedent:

Result: Press ``R`` and see a random camera dolly walk.

We've reached the end of our :class:`.viz.PointViz` tutorial! Thanks and Happy Hacking with the
Ouster Viz library!
