from typing import cast


def osf_list_lidar_metadata(osf_file: str) -> None:
    """List LidarSensor metadata entries using the low-level Reader API."""
    # [doc-stag-osf-reader-metadata-imports]
    # Other imports
    import ouster.sdk.osf as osf
    # [doc-etag-osf-reader-metadata-imports]
    # [doc-stag-osf-reader-metadata]
    reader = osf.Reader(osf_file)
    lidar_metadata = reader.meta_store.find(osf.LidarSensor)
    if not lidar_metadata:
        print("No LidarSensor metadata entries found.")
        return
    for meta_id, lidar_entry in lidar_metadata.items():
        lidar_sensor = cast(osf.LidarSensor, lidar_entry)
        info = lidar_sensor.info
        print(f"meta[{meta_id}] -> sn={info.sn}"
              f", fw_rev={info.fw_rev}, "
              f", prod_line={info.prod_line}")
    # [doc-etag-osf-reader-metadata]


def osf_read_messages(osf_file: str) -> None:
    """Read OSF messages and decode LidarFrame payloads with osf.Reader."""
    import ouster.sdk.osf as osf
    # [doc-stag-osf-reader-messages]
    reader = osf.Reader(osf_file)
    # Read all messages from OSF in timestamp order
    for message in reader.messages():
        print(f"message.ts: {message.ts}"
              f", message.id: {message.id}")
        # In OSF file there maybe different type of messages stored, so here we
        # only interested in LidarFrame messages
        if message.of(osf.LidarFrameStream):
            # Decoding LidarFrame messages
            lidar_frame = message.decode()
            # if decoded successfully just print on the screen LidarFrame
            if lidar_frame is not None:
                print(f"ls = {lidar_frame}")
    # [doc-etag-osf-reader-messages]

