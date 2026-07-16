"""Visualization snippets for using-the-viz.rst."""


def show_lidar_frame(source_url: str, idx: int = 0) -> None:
    """Load and display a single LidarFrame from the given source."""
    # [doc-stag-viz-frame-show]
    from ouster.sdk import open_source
    from ouster.sdk import viz
    # Load a LidarFrame from a given source (could be a pcap file, OSF file, or a live sensor)
    frame_set_source = open_source(source_url)
    # Visualize the LidarFrame
    frames = next(iter(frame_set_source))
    viz.lf_show(frames)
    # When using an indexed source you can do this instead
    # where idx is a valid index value into the source
    viz.lf_show(frame_set_source[idx])
    # [doc-etag-viz-frame-show]


def show_lidar_frames_simultaneously(source_url: str) -> None:
    """Visualize multiple LidarFrames at the same time."""
    from ouster.sdk import open_source
    from ouster.sdk import viz
    frame_set_source = open_source(source_url)
    # [doc-stag-viz-frame-show-list]
    viz.lf_show([frame_set_source[0][0], frame_set_source[1][0], frame_set_source[2][0]])
    # [doc-etag-viz-frame-show-list]


def show_lidar_frames_range(source_url: str, stop: int = 10) -> None:
    """Visualize a range of LidarFrames sequentially."""
    from ouster.sdk import open_source
    from ouster.sdk import viz
    frame_set_source = open_source(source_url)
    # [doc-stag-viz-frame-show-slice]
    viz.lf_show(frame_set_source[0:stop])  # type: ignore[arg-type]
    # [doc-etag-viz-frame-show-slice]
