from typing import cast, Iterable, List, Optional
import click
import logging
import numpy as np
from ouster.cli.plugins.source import source  # type: ignore
from ouster.sdk.core import (FrameSetSource, LidarFrame, restore_instance_ids,
                             FieldClass, FieldType, ChanField)
from ouster.cli.plugins.source_util import (source_multicommand,
                                            SourceCommandType,
                                            SourceCommandContext)
from ouster.sdk.perception import DetectionEngine, ClassicDetectionConfig


logger = logging.getLogger('perception')


@click.command(name="detect")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR_UNREPEATABLE)
def source_detect(ctx: SourceCommandContext) -> None:
    """Run object detection and output results."""
    # TODO: need to get kind && config sourced from command line later
    config = ClassicDetectionConfig()
    detect = DetectionEngine.create(sensor_infos=cast(FrameSetSource, ctx.frame_set_source).sensor_info,
                                    config=config)
    frame_set_iter = ctx.frame_set_iter
    ctx.save_collations = True

    def detect_iter() -> Iterable[List[Optional[LidarFrame]]]:
        if frame_set_iter is None:
            return

        for frames in frame_set_iter():  # type: ignore[operator]
            # TODO: running on multisensor may be wrong for now
            detect.update(frames)
            yield frames

    ctx.frame_set_iter = detect_iter  # type: ignore[assignment]


@click.command(name="restore_instance_ids")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR_UNREPEATABLE)
def source_restore_instance_ids(ctx: SourceCommandContext) -> None:
    """Set instance IDs for each point inside existing objects."""
    frame_set_iter = ctx.frame_set_iter
    ctx.save_collations = True

    def detect_iter() -> Iterable[List[Optional[LidarFrame]]]:
        if frame_set_iter is None:
            return

        for frames in frame_set_iter():  # type: ignore[operator]
            for frame in frames:
                lut = frame.sensor_info.xyzlut_float
                points = lut(frame.field(ChanField.RANGE))
                for object_list_name in frame.objects:
                    ft = FieldType(object_list_name + "::INSTANCE_ID", np.uint32, (), FieldClass.PIXEL_FIELD)
                    instance_ids = frame.add_field(ft)
                    for o in frame.objects[object_list_name]:
                        restore_instance_ids(o, points, instance_ids)

            yield frames

    ctx.frame_set_iter = detect_iter  # type: ignore[assignment]


source.commands['ANY']['detect'] = source_detect
source.commands['ANY']['restore_instance_ids'] = source_restore_instance_ids
