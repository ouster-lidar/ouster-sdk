import numpy as np
from ouster.sdk import core
from ouster.sdk.viz import Mesh, Cloud
from ouster.sdk.viz.widgets import ToggleCloud
from ouster.sdk.zone_monitor import Zrb
from ouster.sdk._bindings.viz import voxel_style_mesh_from_zone_image_pair


def mesh_from_stl(simple_mesh, sensor_info: core.SensorInfo, add_faces=True, add_edges=True):
    mesh = Mesh.from_simple_mesh(simple_mesh)
    mesh.set_edge_rgba((1.0, 0.0, 0.0, 1.0))
    return mesh


def clouds_from_zrb(viz, zrb: Zrb, xyzlut):
    near_image = zrb.near_range_mm
    far_image = zrb.far_range_mm
    near_xyz = xyzlut(near_image)
    near_xyz = near_xyz[near_image > 0]

    far_xyz = xyzlut(far_image)
    far_xyz = far_xyz[far_image > 0]

    near_n_points = near_xyz.shape[0]
    far_n_points = far_xyz.shape[0]
    near_cloud, far_cloud = (Cloud(near_n_points), Cloud(far_n_points))
    near_cloud.set_xyz(near_xyz)
    near_cloud.set_key(np.ones(near_n_points) * 0.25)
    far_cloud.set_xyz(far_xyz)
    far_cloud.set_key(np.ones(far_n_points) * 0.75)
    near_cloud.set_point_size(4)
    far_cloud.set_point_size(4)
    return ToggleCloud(viz, near_cloud), ToggleCloud(viz, far_cloud)


# TODO[tws] c++
def voxel_style_mesh_from_zrb(
    zrb: Zrb,
    metadata: core.SensorInfo,
    voxel_vertices,
    add_faces=True,   # TODO[tws] remove for now
    add_edges=True    # remove for now
):
    mesh = voxel_style_mesh_from_zone_image_pair(
        zrb,
        metadata,
        voxel_vertices
    )
    return mesh


def load_sensor_info(file_path: str):
    with open(file_path) as f:
        return core.SensorInfo(f.read())
