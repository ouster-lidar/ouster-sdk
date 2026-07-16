import sys
from ouster.sdk import open_source
from ouster.sdk import perception


def add_objects_to_source(source_url):
    # [doc-stag-perception-engine]
    # Open a frame source
    source = open_source(source_url)

    # Create a detection engine
    engine = perception.DetectionEngine.create(source.sensor_info)

    # Iterate over the frame sets in the frame source
    frame_counter = 0
    for frame_set in source:
        frame = frame_set[0]

        # Apply the engine to the frame (or the whole frame set.)
        engine.update(frame)

        # Inspect the objects added by the engine.
        print(f'Frame: {frame_counter}')
        frame_counter += 1
        for key, objects in frame.objects.items():
            print(f'Key {key} has {len(objects)} objects.')

    # [doc-etag-perception-engine]


if __name__ == '__main__':
    source_url = sys.argv[1]
    add_objects_to_source(source_url)
