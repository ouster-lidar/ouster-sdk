"""Helper methods for Open3D visualizations."""

import open3d as o3d
import numpy as np
from typing import Tuple


def view_from(vis: o3d.visualization.Visualizer,
              from_point: np.ndarray,
              to_point: np.ndarray = np.array([0, 0, 0])):
    """Helper to setup view direction for Open3D Visualiser.

    Args:
        from_point: camera location in 3d space as ``[x,y,z]``
        to_point: camera view direction in 3d space as ``[x,y,z]``
    """
    ctr = vis.get_view_control()
    up_v = np.array([0, 0, 1])
    dir_v = to_point - from_point
    left_v = np.cross(up_v, dir_v)
    up = np.cross(dir_v, left_v)
    ctr.set_lookat(to_point)
    ctr.set_front(-dir_v)
    ctr.set_up(up)


def create_canvas(w: int, h: int) -> o3d.geometry.TriangleMesh:
    """Create canvas for 2D image.

    Args:
        w: width of the 2D image in screen coords (pixels)
        h: height of the 2D image in screen coords (pixels)
    """
    pict = o3d.geometry.TriangleMesh()
    pict.vertices = o3d.utility.Vector3dVector(np.array([
        [0, 1, 1],
        [0, -1, 1],
        [0, -1, -1],
        [0, 1, -1]
    ]))
    pict.triangles = o3d.utility.Vector3iVector(np.array([
        [0, 3, 2],
        [2, 1, 0],
    ]))
    pict.triangle_uvs = o3d.utility.Vector2dVector(np.array([
        [0.0, 0.0],
        [0.0, 1.0],
        [1.0, 1.0],
        [1.0, 1.0],
        [1.0, 0.0],
        [0.0, 0.0]
    ]))
    im = np.zeros((h, w, 3), dtype=np.float32)
    pict.textures = [o3d.geometry.Image(im)]
    pict.triangle_material_ids = o3d.utility.IntVector(
        np.array([0, 0], dtype=np.int32))
    return pict


def canvas_set_viewport(pic: o3d.geometry.TriangleMesh,
                        viewport: Tuple[float, float, float, float],
                        intrinsics: np.ndarray,
                        extrinsics: np.ndarray,
                        z_near: float = 1.0) -> None:
    """Set the position of the 2D image in space so it seems as static.

    The method should be called on every animation update (animation callback)
    before rendering so the 2D mesh with texture always appear in the position
    that would seem "static" for the observer of the scene through the current
    camera parameters.

    Args:
        pic: canvas with 2D image, created with :func:`.create_canvas`
        viewport: size and position of the canvas on the screen
                  ``[x_pos, y_pos, width, height]``
        intrinsics: current camera intrinsics in a form that is used in Open3D,
                    see :class:`open3d.camera.PinholeCameraIntrinsic.intrinsic_matrix`
                    for details.
        extrinsics: current camera extrinsics used by Open3D. See
                    :class:`open3d.camera.PinholeCameraParameters.extrinsic`
        z_near: the constant near Z plane that is setup in a current Open3D
                ViewControl settings of a Visualizer

    """
    x_pos, y_pos, width, height = viewport
    pict_pos = np.array([[x_pos, y_pos], [x_pos + width, y_pos],
                         [x_pos + width, y_pos + height],
                         [x_pos, y_pos + height]])

    # put canvas in correct camera view (img frame -> camera frame)
    assert intrinsics[0, 0] == intrinsics[1, 1]
    pict_pos = np.append((pict_pos - intrinsics[:2, 2]) / intrinsics[0, 0],
                         np.ones((pict_pos.shape[0], 1)),
                         axis=1)
    pict_pos = pict_pos * (z_near + 0.001)

    # move canvas to world coords (camera frame -> world frame)

    # invert camera extrinsics: inv([R|T]) = [R'|-R'T]
    tr = np.eye(extrinsics.shape[0])
    tr[:3, :3] = np.transpose(extrinsics[:3, :3])
    tr[:3, 3] = - np.transpose(extrinsics[:3, :3]) @ extrinsics[:3, 3]

    pict_pos = tr @ np.transpose(
        np.append(pict_pos, np.ones((pict_pos.shape[0], 1)), axis=1))
    pict_pos = np.transpose(pict_pos / pict_pos[-1])

    # set canvas position
    pict_vertices = np.asarray(pic.vertices)
    np.copyto(pict_vertices, pict_pos[:, :3])


def canvas_set_image_data(pic: o3d.geometry.TriangleMesh,
                          img_data: np.ndarray) -> None:
    """Set 2D image data to 2D canvas.

    Args:
        pic: 2D canvas creates with :func:`.create_canvas`
        img_data: image data RGB (i.e. shape ``[h, w, 3]``)
    """
    pic_img_data = np.asarray(pic.textures[0])
    assert pic_img_data.shape == img_data.shape
    np.copyto(pic_img_data, img_data)
