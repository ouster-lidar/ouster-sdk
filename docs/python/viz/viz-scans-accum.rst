.. _viz-scans-accum:

Visualize SLAM Poses using ``SimpleViz`` - accumulates track, point clouds and map views
-----------------------------------------------------------------------------------------------

.. contents::
   :local:
   :depth: 3


Overview
^^^^^^^^^

Beginning in ouster-sdk 0.13.0, it's easier than ever to use "map" or "scan"
accumulation to visualize lidar scans that contain pose information. The
default visualizer, ``SimpleViz``, present in both ``ouster-cli`` and Ouster
SDK's Python API support these accumulation modes out of the box.

Furthermore, when poses are not present in the ``LidarScan`` accumulation may
still be useful to view the accumulated *N* scans from the live
sensor/recording and reveal the accuracy/repeatability of the data.


Available view modes
~~~~~~~~~~~~~~~~~~~~~

There are three view modes of accumulation implemented in the default
visualizer that may be enabled/disabled depending on its parameters and the data
that is passed through it:

   * **poses** mode, key ``8`` - all scan poses in a trajectory/path view (if poses data is present in scans.)
   * **map accumulation** mode, key ``7`` - overall map view with select ratio of random points
     from every scan (available for scans with or without poses.)
   * **scan accumulation** mode, key ``6`` - accumulated *N* scans (key frames) that is picked
     according to parameters (available for scans with or without poses.)


Key bindings
~~~~~~~~~~~~~

Keyboard controls available with **SimpleViz**:

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

**SimpleViz** is accessible when visualizing data with ``ouster-cli`` using the ``viz`` command.

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

To obtain the densest view use the ``--accum-num N --accum-every 1`` parameters where ``N`` is the
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

   Data fully replayed with map and accum enabled (last current scan is displayed here in gray
   palette)


.. figure:: /images/scans_accum_accum_scan.png

   Data fully replayed with view only last 20 scans accumulated every 10.5 meters


.. figure:: /images/scans_accum_track_all.png

   Data fully replayed with view of only trajectory (yellow knobs is 20 accumulated key frames
   positions)


Programmatic use
^^^^^^^^^^^^^^^^

To use any of these accumulation modes, provide their configuration directly to ``SimpleViz`` via keyword arguments. The following snippet will play back scans from the source ``scans_w_poses`` and the sensor configuration provided by ``meta``::

    import sys
    from ouster.sdk import open_source
    from ouster.sdk.viz import SimpleViz
    from ouster.sdk.mapping import KissBackend

    source_uri = sys.argv[1]
    source = open_source(source_uri)

    kiss_icp = KissBackend([source.metadata])


    def scans_w_poses():
        for scan in source:
            yield kiss_icp.update([scan])


    viz = SimpleViz(
        source.metadata,
        accum_max_num=100,
        accum_min_dist_num=0,
        accum_min_dist_meters=4,
        rate=1,
        on_eof='stop'
    )

    viz.run(scans_w_poses())

Alternatively, ``LidarScanViz`` (which is a lower-level visualizer that implements ``SimpleViz``) can display a static map
from scans that have poses computed in a preprocessing step::

    import sys
    from tqdm import tqdm  # for progress bar
    from ouster.sdk import open_source
    from ouster.sdk.viz import LidarScanViz
    from ouster.sdk.viz.accumulators_config import LidarScanVizAccumulatorsConfig
    from ouster.sdk.mapping import KissBackend

    source_uri = sys.argv[1]
    source = open_source(source_uri)

    kiss_icp = KissBackend([source.metadata])

    num_scans_to_map = 200
    scans_w_poses = [
        kiss_icp.update([scan]) for _, scan in
        zip(tqdm(range(num_scans_to_map), desc="Computing map"), source)
    ]

    viz = LidarScanViz(
        [source.metadata],
        accumulators_config = LidarScanVizAccumulatorsConfig(
            accum_max_num=100,
            accum_min_dist_num=0,
            accum_min_dist_meters=4
        )
    )

    for scan in scans_w_poses:
        viz.update(scan)

    viz.draw(update=True)
    viz.run()
