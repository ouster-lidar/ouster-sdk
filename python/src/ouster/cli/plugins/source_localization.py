import click
import logging
from ouster.cli.plugins.source import source  # type: ignore
import ouster.sdk.client as client
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
@click.option('-v', '--voxel-size', type=float, default=1.4, help="Map voxel size (meters)")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR_UNREPEATABLE)
def source_localize(ctx: SourceCommandContext, map_filename: str, max_range: float, min_range: float,
                    voxel_size: float) -> None:
    """
    Run localization based on the mapping output ply map
    """

    def make_kiss_localization():
        try:
            from ouster.sdk.localization.kiss_localization import KissLocalization
        except ImportError as e:
            raise click.ClickException(click.style("kiss-icp, a package required for slam, is "
                                       f"unsupported on this platform. Error: {str(e)}", fg='red'))

        infos = ctx.scan_source.metadata
        xyz_lut = [client.XYZLut(infos[0], use_extrinsics=True)]
        ctx.misc["localization.map"] = map_filename
        return KissLocalization(
                filename=map_filename,
                xyz_lut=xyz_lut,
                max_range=max_range,
                min_range=min_range,
                voxel_size=voxel_size)

    try:
        localization_engine = make_kiss_localization()
    except (ValueError, click.ClickException) as e:
        logger.error(str(e))
        return

    def localization_iter(scan_source, localization_engine):
        scan_start_ts = None
        for scans in scan_source:
            # Use the first valid scan to check if the iteration restarts
            scan = scans[0]
            if scan is None:
                continue

            scan_ts = client.first_valid_column_ts(scan)
            if scan_ts == scan_start_ts:
                logger.info("Localization engine restarts as scan iteration restarts")
                localization_engine = make_kiss_localization()
            if not scan_start_ts:
                scan_start_ts = scan_ts
            tracked_scans = localization_engine.track(scans)
            yield tracked_scans

    ctx.scan_iter = localization_iter(ctx.scan_iter, localization_engine)


source.commands['ANY']['localize'] = source_localize
