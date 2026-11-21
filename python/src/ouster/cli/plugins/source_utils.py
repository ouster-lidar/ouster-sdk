from typing import Iterable, List, Optional
import click
from ouster.cli.plugins.source import source  # type: ignore
from ouster.sdk.core import (LidarFrame, LidarInfo)
from ouster.cli.plugins.source_util import (source_multicommand,
                                            SourceCommandType,
                                            SourceCommandContext)


@click.command(name="strip")
@click.option("--field", "-f", "fields", multiple=True, type=str, metavar="NAME",
              help="Remove the specified field. Provide multiple times to strip additional fields.")
@click.option("--zm", is_flag=True, help="Strip zone monitor data from sensor info.")
@click.option("--poses", is_flag=True, help="Set poses to identity.")
@click.option("--extrinsics", is_flag=True, help="Set sensor_to_body transforms to identity.")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR_UNREPEATABLE)
def source_strip(ctx: SourceCommandContext, fields: List[str], zm: bool, poses: bool,
                 extrinsics: bool) -> None:
    """Removes fields or other data from a stream."""
    if not zm and not poses and not extrinsics and len(fields) == 0:
        raise click.exceptions.UsageError("Must provide an option to strip.")

    import numpy as np
    frame_set_iter = ctx.frame_set_iter
    assert ctx.frame_set_source is not None
    for si in ctx.frame_set_source.sensor_info:
        # Zone monitoring is lidar-only; extrinsics reset applies to all sensors.
        if zm and isinstance(si, LidarInfo):
            si.zone_set = None
        if extrinsics:
            si.sensor_to_body = np.eye(4)
            si.clear_cache()

    def detect_iter() -> Iterable[List[Optional[LidarFrame]]]:
        if frame_set_iter is None:
            return

        for frames in frame_set_iter():  # type: ignore[operator]
            for name in fields:
                if frames.has_field(name):
                    frames.del_field(name)
            for frame in frames:
                if frame is None:
                    continue
                for name in fields:
                    if frame.has_field(name):
                        frame.del_field(name)
                if poses:
                    frame.body_to_world[:] = np.eye(4)
            yield frames

    if len(fields) > 0 or poses:
        ctx.frame_set_iter = detect_iter  # type: ignore[assignment]


@click.command(name="print")
@click.option("--field", "-f", "fields", multiple=True, type=str, metavar="NAME",
              help="Print the specified field to console. Provide multiple times to"
                   " print additional fields.")
@click.option("--items", "-i", default=1000, type=int, help="Max number of items"
              " after which an array is printed in summarized form. Set to 0 or"
              " less to print all items.")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_print(ctx: SourceCommandContext, fields: List[str], items: int):
    """Print the specified fields to console. If none specified prints all fields."""
    import sys
    import numpy as np
    frame_set_iter = ctx.frame_set_iter
    assert ctx.frame_set_source is not None

    if items <= 0:
        items = sys.maxsize

    def detect_iter() -> Iterable[List[Optional[LidarFrame]]]:
        if frame_set_iter is None:
            return

        def print_fields(data):
            printed = False
            if len(fields) == 0:
                for name in data.fields:
                    print(f"{name}:\n", data.field(name))
                    printed = True
            else:
                for name in fields:
                    if data.has_field(name):
                        print(f"{name}:\n", data.field(name))
                        printed = True
            if printed:
                print("-" * 60)

        for frames in frame_set_iter():  # type: ignore[operator]
            with np.printoptions(threshold=items):
                print_fields(frames)
                for frame in frames:
                    if frame is None:
                        continue
                    print_fields(frame)
            yield frames

    ctx.frame_set_iter = detect_iter  # type: ignore[assignment]


source.commands['ANY']['strip'] = source_strip
source.commands['ANY']['print'] = source_print
