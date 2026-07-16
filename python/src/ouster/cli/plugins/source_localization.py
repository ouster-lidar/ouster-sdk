from typing import cast
import numpy as np
import click
import logging
from functools import partial
from ouster.cli.plugins.source import source  # type: ignore
from ouster.sdk.core import FrameSetSource
from ouster.cli.plugins.source_util import (source_multicommand,
                                            SourceCommandType,
                                            SourceCommandContext)
from ouster.sdk.mapping import LocalizationConfig, LocalizationEngine


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('localization')


@click.command
@click.argument('map_path', required=True, type=str)
@click.option('--max-range', required=False, show_default=True,
              default=150.0, help="Upper limit of range measurments used during localization (meters)")
@click.option('--min-range', required=False, show_default=True,
              default=0.0, help="Lower limit of range measurments used during localization (meters)")
@click.option('-v', '--voxel-size', type=float, help="Map voxel size (meters)")
@click.option('--max-iterations', required=False, show_default=True,
              default=500, type=int,
              help="Maximum number of ICP registration iterations")
@click.option('--deskew-method', type=click.Choice(['auto', 'none', 'constant_velocity', 'imu_deskew']),
              default='auto', show_default=True,
              help="Method used for motion compensation (deskewing) of point clouds")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR_UNREPEATABLE)
def source_localize(ctx: SourceCommandContext, map_path: str, max_range: float, min_range: float,
                    voxel_size: float, max_iterations: int, deskew_method: str) -> None:
    """
    Run localization based on an existing pointcloud map
    """

    if max_iterations <= 0:
        raise click.UsageError("max_iterations must be greater than 0")

    def make_kiss_localization() -> LocalizationEngine:

        config = LocalizationConfig.create("lio")
        config.deskew_method = deskew_method
        config.min_range = min_range
        config.max_range = max_range
        config.voxel_size = voxel_size if voxel_size is not None else 0.0
        config.max_iterations = max_iterations
        config.initial_pose = ctx.other_options.get("initial_pose", np.eye(4))

        ctx.misc["localization.map"] = map_path
        return LocalizationEngine.create(
                cast(FrameSetSource, ctx.frame_set_source).sensor_info,
                map_path,
                config)

    def localization_iter(frame_set_source):
        localization_engine = make_kiss_localization()
        for frames in frame_set_source():
            yield localization_engine.update(frames)

    # type ignored because generators are tricky to mypy
    ctx.frame_set_iter = partial(localization_iter, ctx.frame_set_iter)  # type: ignore


source.commands['ANY']['localize'] = source_localize
