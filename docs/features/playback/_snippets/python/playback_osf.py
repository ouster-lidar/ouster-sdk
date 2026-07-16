"""Python snippets for OSF playback docs."""

def osf_create_frames(osf_file: str) -> None:
    """Create a new OSF file and write a single blank frame."""
    # [doc-stag-osf-create-imports]
    import ouster.sdk.core as core
    import ouster.sdk.osf as osf
    # [doc-etag-osf-create-imports]
    # [doc-stag-osf-create]
    # Create sensor info (single sensor) and writer
    info = core.SensorInfo.from_default(core.LidarMode._512x10)
    writer = osf.Writer(osf_file, info)
    # Allocate an empty frame that matches the metadata
    frame = core.LidarFrame(info)
    # Manipulate the frame here if desired
    # Write the frame to stream 0
    writer.save(0, frame)
    # [doc-etag-osf-create]
    writer.close()


def osf_write_frames(hostname: str, output_file: str, n_frames: int = 100) -> None:
    """Read frames from a live sensor and store them in an OSF file."""
    # [doc-stag-osf-write-imports]
    # Other imports
    from ouster.sdk import open_source
    import ouster.sdk.osf as osf
    # [doc-etag-osf-write-imports]
    # [doc-stag-osf-write-setup]
    source = open_source(hostname)
    info = source.sensor_info[0]
    # [doc-etag-osf-write-setup]
    # [doc-stag-osf-write]
    writer = osf.Writer(output_file, info)
    written = 0

    for frame_set in source:
        for idx, frame in enumerate(frame_set):
            if frame is None: continue # noqa
            writer.save(idx, frame)
        written += 1
        if written >= n_frames: break # noqa
    # [doc-etag-osf-write]

    writer.close()
    source.close()


__all__ = ["osf_create_frames", "osf_write_frames"]
