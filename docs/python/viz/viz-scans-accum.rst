.. _viz-scans-accum:

Visualize SLAM Poses using ``ScansAccumulator`` - accumulates track, point clouds and map views
-----------------------------------------------------------------------------------------------

.. contents::
   :local:
   :depth: 3


Overview
^^^^^^^^^

**ScansAccumulator** is the continuation of the efforts to view the lidar data with poses that may
come within the ``LidarScan.pose`` property. When poses are not present in the ``LidarScan``
**ScansAccumulator** may still be useful to view the accumulated *N* scans from the live
sensor/recording and reveal the accuracy/repeatability of the data.


Available view modes
~~~~~~~~~~~~~~~~~~~~~

There are three view modes of **ScansAccumulator**, that may be enabled/disabled depending on
it's params and the data that is passed throught it:

   * **poses** (or **TRACK**), key ``8`` - all scan poses in a trajectory/path view (available only
     if poses data is present in scans)
   * **scan map** (or **MAP**), key ``7`` - overall map view with select ratio of random points
     from every scan (available for scans with/without poses)
   * **scan accum** (or **ACCUM**), key ``6`` - accumulated *N* scans (key frames) that is picked
     according to params (available for scans with/without poses)


Key bindings
~~~~~~~~~~~~~

Keyboard controls available with **ScansAccumulator**:

    ==============  =============================================================
        Key         What it does
    ==============  =============================================================
    ``6``           Toggle scans accumulation view mode (ACCUM)
    ``7``           Toggle overall map view mode (MAP)
    ``8``           Toggle poses/trajectory view mode (TRACK)
    ``k / K``       Cycle point cloud coloring mode of accumulated clouds or map
    ``g / G``       Cycle point cloud color palette of accumulated clouds or map
    ``j / J``       Increase/decrease point size of accumulated clouds or map
    ==============  =============================================================



Use in Ouster SDK CLI
^^^^^^^^^^^^^^^^^^^^^^

**ScansAccumulator** is accessible when visualizing data with ``ouster-cli`` using the ``viz`` command.

Here are the ``viz`` command options that affect it:

  * ``--accum-num INTEGER`` - Accumulate up to this number of past scans for visualization.
    Use <= 0 for unlimited. Defaults to 100 if ``--accum-every`` or ``--accum-every-m`` is set.
  * ``--accum-every INTEGER`` - Add a new scan to the accumulator every this number of scans.
  * ``--accum-every-m FLOAT`` - Add a new scan to the accumulator after this many meters of travel.
  * ``--map`` - If set, add random points from every scan into an overall map for visualization.
    Enabled if either ``--map-ratio`` or ``--map-size`` are set.
  * ``--map-ratio R`` - Fraction of random points in every scan to add to overall map (0, 1]. [default: 0.01]
  * ``--map-size N`` - Maximum number of points in overall map before discarding. [default: 1500000]


Examples of the CLI commands:

Dense accumulated clouds view (with every point of a scan)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To obtain the densest view use the ``--accum-num N --accum-every 1`` params where ``N`` is the
number of clouds to accumulate (``N`` up to 100 is generally small enough to avoid slowing down the
viz interface)::

   ouster-cli source OS-1-128_v3.0.1_1024x10_20230216_142857-000.pcap slam viz --accum-num 20

and the dense accumulated clouds result:

.. figure:: /images/scans_accum_dense_every.png

   Dense view of 20 accumulated scans during the ``slam viz`` run


Overall map view (with poses)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

One of the main task that we often needed was a preview of the overall map from the OSF with poses
for example::

   ouster-cli source OS-0-128_v3.0.1_1024x10_20230415_152307-000.osf viz --accum-num 20 \
   --accum-every 0 --accum-every-m 10.5 --map -e stop


And here is the final result when viz is done and stopped (``-e stop``) after playing the whole file:

.. figure:: /images/scans_accum_map_all_scan.png

   Data fully replayed with map and accum enabled (last current scan is displayed here in grey
   palette)


.. figure:: /images/scans_accum_accum_scan.png

   Data fully replayed with view only last 20 scans accumulated every 10.5 meters


.. figure:: /images/scans_accum_track_all.png

   Data fully replayed with view of only trajectory (yellow knobs is 20 accumulated key frames
   positions)


Programmatic use with (and without) PointViz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

With ``point_viz: PointViz`` object the ``ScansAccumulator`` can be used as a regular
``LidarScanViz`` and passed directly to ``SimpleViz``::

   from ouster.sdk.viz import PointViz, add_default_controls, ScansAccumulator, SimpleViz

   point_viz = PointViz("SimpleViz usecase")
   add_default_controls(point_viz)

   # ... get scans_w_poses Scans source ...

   scans_acc = ScansAccumulator(meta,
                                point_viz=point_viz,
                                accum_max_num=10,
                                accum_min_dist_num=1,
                                map_enabled=True,
                                map_select_ratio=0.5)

   SimpleViz(scans_acc, rate=1.0).run(scans_w_poses)


Alternatively with a ``PointViz`` it can be used as a canvas to draw the final state only::

   from ouster.sdk.viz import ScansAccumulator, add_default_controls, PointViz

   point_viz = PointViz("Overall map case")
   add_default_controls(point_viz)

   # ... get scans_w_poses Scans source ...

   scans_acc = ScansAccumulator(meta,
                                point_viz=point_viz,
                                accum_max_num=10,
                                accum_min_dist_num=1,
                                map_enabled=True,
                                map_select_ratio=0.5)

   for scan in scans_w_poses:
       scans_acc.update(scan)

   scans_acc.draw(update=True)
   point_viz.update()
   point_viz.run()


Without ``PointViz`` it can be used as in the following snippet to accumulate all data and use the
data later to draw anywhere (here we still use the ``PointViz`` and ``viz.Cloud()`` as a main
graphing tool, but it can be ``matplotlib`` instead)::

   from ouster.sdk.viz import grey_palette, ScansAccumulator, Cloud, add_default_controls, PointViz

   # ... get scans_w_poses Scans source ...

   # create scans accum without PointViz
   scans_acc = ScansAccumulator(meta,
                                map_enabled=True,
                                map_select_ratio=0.5)

   # processing doesn't require viz presence in scans accum
   for scan in scans_w_poses:
       scans_acc.update(scan)

   point_viz = PointViz("Standalone case")
   add_default_controls(point_viz)

   # draw the cloud manually to the viz using ScansAccumulator MAP data
   cloud_map = Cloud(scans_acc._map_xyz.shape[0])
   cloud_map.set_xyz(scans_acc._map_xyz)
   cloud_map.set_key(scans_acc._map_keys["NEAR_IR"])
   cloud_map.set_palette(grey_palette)
   cloud_map.set_point_size(1)
   point_viz.add(cloud_map)

In the example above one might use ``matplotlib`` with some modifications to use palette for picking
the key color.

