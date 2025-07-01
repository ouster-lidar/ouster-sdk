from typing import cast, List, Optional
import click
import logging
from functools import partial
from ouster.cli.plugins.source import source  # type: ignore
from ouster.sdk.core import LidarScan, ScanSource
from ouster.cli.plugins.source_util import (source_multicommand,
                                            SourceCommandType,
                                            SourceCommandContext)


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('localization')


@click.command
@click.argument('map_filename', required=True, type=str)
@click.option('--max-range', required=False, show_default=True,
              default=150.0, help="Upper limit of range measurments used during localization (meters)")
@click.option('--min-range', required=False, show_default=True,
              default=0.0, help="Lower limit of range measurments used during localization (meters)")
@click.option('-v', '--voxel-size', type=float, help="Map voxel size (meters)")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR_UNREPEATABLE)
def source_localize(ctx: SourceCommandContext, map_filename: str, max_range: float, min_range: float,
                    voxel_size: float) -> None:
    """
    Run localization based on the mapping output ply map
    """

    try:
        from ouster.sdk.mapping import LocalizationConfig, LocalizationEngine
        from ouster.sdk.mapping.util import determine_voxel_size
    except ImportError as e:
        raise click.ClickException(click.style("kiss-icp, a package required for slam, is "
                                    f"unsupported on this platform. Error: {str(e)}", fg='red'))

    def make_kiss_localization() -> LocalizationEngine:

        def live_sensor_voxel_size(scans: List[Optional[LidarScan]]) -> Optional[float]:
            """a customized version of determine_voxel_size for live sensors"""
            voxel_size = determine_voxel_size(scans)
            if voxel_size is not None:
                if cast(ScanSource, ctx.scan_source).is_live:
                    logger.info("Choosing a larger voxel size to support real-time processing of live sensors.")
                    voxel_size *= 2.2 * voxel_size
                logger.info(f"voxel-size arg is not set, using an estimated value of {voxel_size:.4g} m.")
            return voxel_size

        config = LocalizationConfig()
        config.min_range = min_range
        config.max_range = max_range
        config.voxel_size = voxel_size if voxel_size is not None else live_sensor_voxel_size
        config.initial_pose = ctx.other_options["initial_pose"]
        config.backend = "kiss"

        ctx.misc["localization.map"] = map_filename
        return LocalizationEngine(
                infos=cast(ScanSource, ctx.scan_source).sensor_info,
                config=config,
                map=map_filename)

    def localization_iter(scan_source):
        localization_engine = make_kiss_localization()
        for scans in scan_source():
            scan = scans[0]
            if scan is None:
                continue

            yield localization_engine.update(scans)

    # type ignored because generators are tricky to mypy
    ctx.scan_iter = partial(localization_iter, ctx.scan_iter)  # type: ignore


source.commands['ANY']['localize'] = source_localize
