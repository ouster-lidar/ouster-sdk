# flake8: noqa: W293
import argparse
import os


def osf_read_frames(osf_file: str, limit: int = 10) -> None:
    """Read Lidar Frames from an OSF file.

    Shows frames for each sensor in time order stored in the OSF File.

    Args:
        osf_file: path to osf file.
    """
    # [doc-stag-osf-read-imports]
    import ouster.sdk.osf as osf
    # [doc-etag-osf-read-imports]
    # [doc-stag-osf-read-frames]
    source = osf.OsfFrameSetSource(osf_file)
    count = 0

    for frame_set in source:
        for frame in frame_set:
            if frame is None:
                continue
            print(f'frame = {frame},'
                  f' WxH={frame.w}x{frame.h}')
            count += 1
            if count >= limit:
                return
    source.close()
    # [doc-etag-osf-read-frames]


def osf_get_sensors_info(osf_file: str) -> None:
    """Read Lidar Sensors info from an OSF file.

    Shows metadata for all sensors found in an OSF file.

    Args:
        osf_file: path to osf file.
    """
    # [doc-stag-osf-get-sensors-info]
    from ouster.sdk import osf
    frames = osf.OsfFrameSetSource(osf_file)
    for sensor_id, info in enumerate(frames.sensor_info):
        print(f"sensor[{sensor_id}] = ", info)
    frames.close()
    # [doc-etag-osf-get-sensors-info]


def osf_slice_frames(osf_file: str) -> str:
    """Copy frames from input OSF file with reduction using the Writer API.

    Slicing is done via saving only RANGE, SIGNAL and REFLECTIVITY fields into an output OSF files.
    """
    from ouster.sdk import core
    import ouster.sdk.osf as osf

    def make_sliced_fname(f: str) -> str:
        """Generate sliced filename and remove it if it exists."""
        output_file = os.path.splitext(os.path.basename(f))[0] + '_sliced.osf'
        if os.path.exists(output_file):
            os.remove(output_file)
        return output_file

    output_file = make_sliced_fname(osf_file)

    # Frames reader from input OSF
    # New field types should be a subset of fields in encoded LidarFrame so we just assume that
    # RANGE, SIGNAL and REFLECTIVITY fields will be present in the input OSF file.
    # [doc-stag-osf-slice-frames]
    frames = osf.OsfFrameSetSource(osf_file)
    fields_to_write = [
        core.ChanField.RANGE,
        core.ChanField.SIGNAL,
        core.ChanField.REFLECTIVITY]

    writer = osf.Writer(output_file,
                        frames.sensor_info,
                        fields_to_write)
    # Read frames and write back
    for frame_set in frames:
        for idx, frame in enumerate(frame_set):
            if frame is None:
                continue
            print(f"writing sliced frame with ts = \
                   {frame.get_first_valid_packet_timestamp()}")
            writer.save(idx, frame)
    writer.close()
    # [doc-etag-osf-slice-frames]
    return output_file


def add_objects_to_frame(frame=None):
    """Populate a LidarFrame with example Object instances.

    Args:
        frame: optional LidarFrame to update directly.
    """
    if frame is None:
        from ouster.sdk import core
        sensor_info = core.SensorInfo.from_default(core.LidarMode._1024x10)
        frame = core.LidarFrame(sensor_info)

    print(f"Original frame = {frame}, WxH={frame.w}x{frame.h}")
    # [doc-stag-add-object]
    from ouster.sdk import core
    import numpy as np
    objects = [core.Object(), core.Object()]
    objects[0].id = 1
    objects[0].creation_ts = 99
    objects[0].timestamp = 199
    objects[0].class_id = 1
    objects[0].class_confidence = 0.9
    objects[0].object_to_body.position = np.array([1, 2, 3])
    objects[0].object_to_body.set_rotation(np.array([2, 2, 2]))
    objects[0].body_to_world.position = np.array([10, 20, 30])
    objects[0].body_to_world.set_rotation(np.array([0.1, 0.2, 0.3]))
    objects[0].velocity = np.array([2, 3, 4])
    objects[0].dimensions = np.array([1, 1, 1])
    objects[0].properties["num_points"] = '[100]'
    objects[0].properties["attributes"] = '["eats_icecream", "carries_bag"]'
    objects[1].id = 2
    objects[1].creation_ts = 100
    objects[1].timestamp = 200
    objects[1].class_id = 2
    objects[1].class_confidence = 0.8
    objects[1].object_to_body.position = np.array([3, 2, 1])
    objects[1].object_to_body.set_rotation(np.array([1, 1, 1]))
    objects[1].body_to_world.position = np.array([4, 5, 6])
    objects[1].body_to_world.set_rotation(np.array([0.4, 0.5, 0.6]))
    objects[1].velocity = np.array([4, 3, 2])
    objects[1].dimensions = np.array([2, 2, 2])
    objects[1].properties["num_points"] = '[50]'
    objects[1].properties["attributes"] = '["parked_illegaly"]'
    frame.objects["test_objects"] = objects
    # [doc-etag-add-object]
    print(f"Added {len(objects)} objects to frame under key 'test_objects'.")
    print(f"Updated frame = {frame}, objects keys = {list(frame.objects.keys())}")
    return objects


def write_classmaps(osf_path, sensor_info=None):
    """Write ClassMap metadata to an OSF file."""
    import ouster.sdk.osf as osf
    from ouster.sdk import core
    input_source = None
    auto_close_writer = False
    output_path = osf_path
    if sensor_info is None:
        input_source = osf.OsfFrameSetSource(str(osf_path))
        if not input_source.sensor_info:
            input_source.close()
            raise ValueError(f"No sensor metadata found in {osf_path}")
        sensor_info = input_source.sensor_info[0]
        output_path = os.path.splitext(os.path.basename(str(osf_path)))[0] + '_classmaps.osf'
        auto_close_writer = True
    # [doc-stag-write-classmaps]
    writer = osf.Writer(str(output_path), [sensor_info])

    class_map1 = core.ClassMap({
        1: 'dog',
        2: 'cat'
    })

    class_map2 = core.ClassMap({
        1: 'tree',
        2: 'bush'
    })

    class_maps = core.ClassMapSet({
        'four_legs': class_map1,
        'zero_legs': class_map2
    })

    metadata = core.FrameSetSourceMetadataSet()
    metadata['class_maps'] = class_maps
    metadata['additional_info'] = "Test additional info"
    
    
    writer.save(metadata)
    # [doc-etag-write-classmaps]
    print(f"Wrote class map metadata to {output_path}")
    if auto_close_writer:
        writer.close()
    if input_source is not None:
        input_source.close()
    return writer, class_maps, metadata


def read_classmaps(osf_path):
    """Read ClassMap metadata from an OSF FrameSetSource."""
    # [doc-stag-read-classmaps]
    from ouster.sdk import open_source
    src = open_source(str(osf_path))
    # Unlike C++, no .is<>) check is needed: metadata() returns a
    # ClassMapSet directly (the bindings unwrap the stored type for you).
    class_map_set = src.metadata('class_maps')
    dog_class = class_map_set['four_legs'][1]
    # [doc-etag-read-classmaps]
    print(f"Example class label for key 1 in 'four_legs': {dog_class}")
    src.close()
    return src, class_map_set, dog_class


def main():
    """OSF examples runner."""
    examples = {
        "read-frames": osf_read_frames,
        "slice-frames": osf_slice_frames,
        "get-sensors-info": osf_get_sensors_info,
        "add-objects-to-frame": add_objects_to_frame,
        "write-classmaps": write_classmaps,
        "read-classmaps": read_classmaps,
    }

    description = "Ouster Python SDK OSF examples. The EXAMPLE must be one of:\n  " + str.join(
        '\n  ', examples.keys())

    parser = argparse.ArgumentParser(
        description=description, formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument('osf_path', metavar='OSF', nargs='?', help='path to osf file')
    parser.add_argument('example',
                        metavar='EXAMPLE',
                        choices=examples.keys(),
                        help='name of the example to run')
    args = parser.parse_args()

    try:
        example = examples[args.example]
    except KeyError:
        print(f"No such example: {args.example}")
        print(description)
        exit(1)

    print(f'example: {args.example}')

    if args.example != "add-objects-to-frame" and not args.osf_path:
        parser.error("OSF path is required for this example.")

    if args.example == "add-objects-to-frame":
        example()
        return

    example(args.osf_path)


if __name__ == "__main__":
    main()
