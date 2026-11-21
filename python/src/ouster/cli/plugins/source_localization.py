from typing import cast
import click
import logging
from functools import partial
from ouster.cli.plugins.source import source  # type: ignore
from ouster.sdk.core import FrameSetSource, VHMInternalStorageMode
from ouster.cli.plugins.source_util import (source_multicommand,
                                            SourceCommandType,
                                            SourceCommandContext)
from ouster.sdk.mapping import LocalizationConfig, LocalizationEngine


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('localization')


_voxel_storage_mode_lut = {x.name.lower(): x for x in VHMInternalStorageMode}


@click.command
@click.argument('map_path', required=True, type=str)
@click.option('--max-range', required=False, show_default=True,
              default=150.0, help="Upper limit of range measurements used during localization (meters)")
@click.option('--min-range', required=False, show_default=True,
              default=0.0, help="Lower limit of range measurements used during localization (meters)")
@click.option('-v', '--voxel-size', type=float, help="Voxel grid resolution used for downsampling (meters)")
@click.option('--voxel-map-size', required=False, show_default=True,
              default=float('inf'), help="Maximum distance from the initial pose at which "
                                         "points are retained in the local voxel map (meters). "
                                         "Defaults to infinity, i.e. the preloaded map is not trimmed.")
@click.option('--max-iterations', required=False, show_default=True,
              default=500, type=int,
              help="Max number of ICP registration iterations")
@click.option('--deskew-method', type=click.Choice(['auto', 'none', 'constant_velocity', 'imu_deskew']),
              default='auto', show_default=True,
              help="Method used for motion compensation (deskewing) of point clouds")
@click.option('--plumb-method', type=click.Choice(['imu', 'walls', 'all']),
              default='all', show_default=True,
              help="How the first frame is plumbed with IMU deskewing to set the initial gravity. "
                   "imu: gravity from the IMU. walls: gravity from wall geometry only, no IMU. all: "
                   "the IMU estimate refined by wall geometry, keeping the IMU-only result when the "
                   "scene has too little vertical structure. The wall search is expensive, so it "
                   "stops once the first frame set produces a pose; frame sets dropped before "
                   "that, e.g. with no valid points, are plumbed again.")
@click.option('--voxel-storage-mode', type=click.Choice(list(_voxel_storage_mode_lut.keys())),
              default=VHMInternalStorageMode.AUTO.name.lower(), show_default=True, required=False,
              help="Select data type for internal voxel storage. Auto will select the smallest representation "
              "suitable for the voxel size")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR_UNREPEATABLE)
def source_localize(ctx: SourceCommandContext, map_path: str, max_range: float, min_range: float,
                    voxel_size: float, voxel_map_size: float, max_iterations: int, deskew_method: str,
                    plumb_method: str, voxel_storage_mode: str) -> None:
    """
    Run localization based on an existing pointcloud map
    """

    if max_iterations <= 0:
        raise click.UsageError("max_iterations must be greater than 0")

    if voxel_map_size <= 0:
        raise click.UsageError("voxel_map_size must be greater than 0")

    def make_lio_localization() -> LocalizationEngine:

        config = LocalizationConfig.create("lio")
        config.deskew_method = deskew_method
        config.min_range = min_range
        config.max_range = max_range
        config.voxel_size = voxel_size if voxel_size is not None else 0.0
        config.voxel_map_size = voxel_map_size
        config.max_iterations = max_iterations
        config.initial_pose = ctx.other_options.get("initial_pose")
        config.plumb_method = plumb_method
        config.voxel_storage_mode = _voxel_storage_mode_lut[voxel_storage_mode]

        src = cast(FrameSetSource, ctx.frame_set_source)
        ctx.misc["localization.map"] = map_path
        return LocalizationEngine.create(
                src.sensor_info,
                map_path,
                config)

    def localization_iter(frame_set_source):
        localization_engine = make_lio_localization()
        for frames in frame_set_source():
            yield localization_engine.update(frames)

    # type ignored because generators are tricky to mypy
    ctx.frame_set_iter = partial(localization_iter, ctx.frame_set_iter)  # type: ignore


source.commands['ANY']['localize'] = source_localize
