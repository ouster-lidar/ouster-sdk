Visualizing LidarFrames
======================

The Ouster visualization toolkit is written in C++ with Python bindings for Python functionality. It
consists of the following:

- ``simple-viz`` (:py:class:`.viz.SimpleViz`): the default Python application visualizer, which can
  also be used as an entrypoint for more sophisticated custom point cloud visualizations
- ``ouster_viz``: the core C++ library 
- ``ouster.sdk.viz``: the Python module for the bindings


Using ``ouster-cli`` is a fastest way to visualize data from a connected sensor, recorded PCAP,
or OSF files with SLAM poses. Refer to :doc:`/getting-started/index` for details.


Programmatic use
----------------

If you want to visualize a ``LidarFrame`` or a list of ``LidarFrame`` objects using the python API, you can use
:ouster:func:`lf_show <py=ouster.sdk.viz.core.lf_show>` method. Here is a code snippet that shows how it can be used to
visualize a list of LidarFrame:

.. literalinclude:: _snippets/python/viz_frame_show_example.py
   :language: python
   :start-after: [doc-stag-viz-frame-show-open-source]
   :end-before: [doc-etag-viz-frame-show-open-source]
   :dedent: 0
   :class: doc-snippet
   :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/viz_frame_show_example.py>`__


This will open an interactive window displaying the :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` as PointCloud.


Additionally, :ouster:func:`lf_show <py=ouster.sdk.viz.core.lf_show>` accepts a list or slice of LidarFrame(s), the following snippet shows this use:


.. literalinclude:: _snippets/python/viz_frame_show_example.py
   :language: python
   :start-after: [doc-stag-viz-frame-show-slice]
   :end-before: [doc-etag-viz-frame-show-slice]
   :dedent: 0
   :class: doc-snippet
   :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/python/viz_frame_show_example.py>`__


In the first example will pass a list of frames to ``lf_show``, these frames will appear in SimpleViz window
at the same time, you can toggle each frame on/off within SimpleViz using ``CTRL+1`` through ``CTRL+9`` which is very helpful
to compare frames side by side. It is worth noting that the frames don't need to be from the same source.

In the second example, we are passing a slice of frames to ``lf_show``, this will visualize the frames
within the slice sequentially. You can press the ``spacebar`` to unpause the playback or use the 
`<`, `>` keys to step back and forth between frames. 

For more advanced visualization you can explore a programmatically accessible :py:class:`.viz.PointViz`
API below:

.. figure:: /images/viz-tutorial/lidar_frame_viz_checkers.png
    :align: center

    ``PointViz`` with ``Image``, ``Label`` and masks applied
