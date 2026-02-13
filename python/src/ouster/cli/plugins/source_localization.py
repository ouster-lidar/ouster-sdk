from typing import cast
import numpy as np
import click
import logging
from functools import partial
from ouster.cli.plugins.source import source  # type: ignore
from ouster.sdk.core import ScanSource
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
@click.option('--deskew-method', type=click.Choice(['auto', 'none', 'constant_velocity', 'imu_deskew']),
              default='auto', show_default=True,
              help="Method used for motion compensation (deskewing) of point clouds")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR_UNREPEATABLE)
def source_localize(ctx: SourceCommandContext, map_path: str, max_range: float, min_range: float,
                    voxel_size: float, deskew_method: str) -> None:
    """
    Run localization based on the mapping output ply map
    """

    def make_kiss_localization() -> LocalizationEngine:

        config = LocalizationConfig()
        config.backend = "kiss"
        config.deskew_method = deskew_method
        config.min_range = min_range
        config.max_range = max_range
        config.voxel_size = voxel_size if voxel_size is not None else 0.0
        config.initial_pose = ctx.other_options.get("initial_pose", np.eye(4))

        ctx.misc["localization.map"] = map_path
        return LocalizationEngine(
                infos=cast(ScanSource, ctx.scan_source).sensor_info,
                config=config,
                map=map_path)

    def localization_iter(scan_source):
        localization_engine = make_kiss_localization()
        for scans in scan_source():
            yield localization_engine.update(scans)

    # type ignored because generators are tricky to mypy
    ctx.scan_iter = partial(localization_iter, ctx.scan_iter)  # type: ignore


source.commands['ANY']['localize'] = source_localize
