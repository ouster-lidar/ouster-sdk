# type: ignore

import click
from ouster.cli.plugins.io_type import OusterIoType
from ouster.cli.plugins.source_osf import source
from ouster.cli.plugins.source import _source_arg_name, _output_file_arg_name
from ouster.sdkx.mapping import mapping
from ouster.cli.core.util import click_ro_file


@click.command
@click.argument(_output_file_arg_name, required=True)
@click.option('-d', '--min_dist', default=2.0, help="min dist (m) for points to "
        "save. Default value is 2m")
@click.option('-s', '--voxel_size', default=0.1, help="voxel map size for down "
        "sampling. This is same with open3D voxel size. Default value is 0.1. "
        "The bigger the value, the fewer points output"
        " http://www.open3d.org/docs/0.6.0/python_api/open3d.geometry.voxel_down_sample.html")
@click.option('-f',
    '--field',
    required=False,
    type=click.Choice(['SIGNAL',
    'NEAR_IR',
    'REFLECTIVITY'],
    case_sensitive=False),
    default="REFLECTIVITY",
    help="chanfield for output file key value. Choose between SIGNAL, NEAR_IR, "
    "REFLECTIVITY. Default field is REFLECTIVITY")
@click.option('--print_process', required=False, type=bool, default=True, help="Default is On")
@click.option('--verbose_print', required=False, type=bool, default=False,
              help="Print point cloud status much frequently. Default is Off")
@click.pass_context
def convert_from_osf(ctx, *args, **kwargs):
    """
    Takes in an OSF file, perturbs the point cloud data using a saved SLAM trajectory,
    and saves the modified point cloud into a new file.
    Output file format depends on output filename extension. The valid output files
    extention are .las, .ply and .pcd. For large point cloud, the output will
    be split into multiple files and each file is around 1G.
    """
    kwargs['input_file'] = ctx.obj.get(_source_arg_name)

    ctx.forward(mapping.point_cloud_convert, *args, **kwargs)


source.commands[OusterIoType.OSF]['convert'].conversions[OusterIoType.PLY] = convert_from_osf
source.commands[OusterIoType.OSF]['convert'].conversions[OusterIoType.PCD] = convert_from_osf
source.commands[OusterIoType.OSF]['convert'].conversions[OusterIoType.LAS] = convert_from_osf


@click.command
@click.argument('viz', required=False, default="", type=str)
@click.option('-m', '--meta', required=False, default=None, type=click_ro_file,
              help="metadata file for pcap input")
@click.option('--slam_name', default='kiss_slam', help="slam name")
@click.option('-l', '--lidar_port', default=7502, help="Lidar port")
@click.option('-i', '--imu_port', default=7503, help="IMU port")
@click.option('-o', '--output', required=False, help="OSF output filename")
@click.pass_context
def run_slam(ctx, *args, **kwargs) -> None:
    """
    slam command runs SLAM with an optinal visualizer

    Users have the option to choose between a live sensor or a recorded pcap.
    The output of the SLAM process consists of poses, which can be further
    processed using an algorithm to generate perturbed poses for each lidar
    scan column. These overwritten lidar scans are be saved in an OSF file for later use.

    To turn on visualizer, append 'viz' or 'visualizer' to the command, case insensitive.
    """
    source = ctx.obj.get(_source_arg_name)
    kwargs['hostname'] = source
    ctx.forward(mapping.run_slam, *args, **kwargs)


source.commands[OusterIoType.PCAP]['slam'] = run_slam
source.commands[OusterIoType.SENSOR]['slam'] = run_slam
