from typing import cast, Iterable, List, Optional
import click
import logging
import numpy as np
from ouster.cli.plugins.source import source  # type: ignore
from ouster.sdk.core import (Frame, FrameSetSource, restore_instance_ids,
                             FieldClass, FieldType, ChanField)
from ouster.cli.plugins.source_util import (source_multicommand,
                                            SourceCommandType,
                                            SourceCommandContext)
from ouster.sdk.perception import DetectionEngine, ClassicDetectionConfig


logger = logging.getLogger('perception')

_default_classic_config = ClassicDetectionConfig()


@click.command(name="detect")
@click.option('--save-instance-id-fields/--no-save-instance-id-fields',
              default=_default_classic_config.save_instance_id_fields, show_default=True,
              help="Save per-pixel instance id fields for detected objects")
@click.option('--cluster-filter-min-side-length', type=float, show_default=True,
              default=round(_default_classic_config.cluster_filter_min_side_length, 6),
              help="Minimum side length a cluster must have in order to be kept (meters)")
@click.option('--cluster-filter-max-side-length', type=float, show_default=True,
              default=round(_default_classic_config.cluster_filter_max_side_length, 6),
              help="Maximum side length a cluster may have in order to be kept (meters). "
                   "Ignored when negative")
@click.option('--cluster-filter-min-vertical-size', type=float, show_default=True,
              default=round(_default_classic_config.cluster_filter_min_vertical_size, 6),
              help="Minimum size a cluster must have along the vertical dimension (meters). "
                   "Filters out very flat objects that may be outliers")
@click.option('--cluster-filter-max-volume', type=float, show_default=True,
              default=round(_default_classic_config.cluster_filter_max_volume, 6),
              help="Maximum volume a cluster may have in order to be kept (cubic meters). "
                   "Ignored when negative")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR_UNREPEATABLE)
def source_detect(ctx: SourceCommandContext, save_instance_id_fields: bool,
                  cluster_filter_min_side_length: float, cluster_filter_max_side_length: float,
                  cluster_filter_min_vertical_size: float, cluster_filter_max_volume: float) -> None:
    """Run object detection and output results."""
    config = ClassicDetectionConfig()
    config.save_instance_id_fields = save_instance_id_fields
    config.cluster_filter_min_side_length = cluster_filter_min_side_length
    config.cluster_filter_max_side_length = cluster_filter_max_side_length
    config.cluster_filter_min_vertical_size = cluster_filter_min_vertical_size
    config.cluster_filter_max_volume = cluster_filter_max_volume
    detect = DetectionEngine.create(
        sensor_infos=list(cast(FrameSetSource, ctx.frame_set_source).sensor_info),
        config=config)
    frame_set_iter = ctx.frame_set_iter
    ctx.save_collations = True

    def detect_iter() -> Iterable[List[Optional[Frame]]]:
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

    def detect_iter() -> Iterable[List[Optional[Frame]]]:
        if frame_set_iter is None:
            return

        for frames in frame_set_iter():  # type: ignore[operator]
            for frame in frames.valid_frames():
                if frame.sensor_info is None or not frame.has_field(ChanField.RANGE):
                    continue
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
