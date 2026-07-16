"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Types for managing visualization state.
"""

import copy
import threading
import weakref
from queue import LifoQueue
from dataclasses import dataclass
from functools import partial
import numpy as np
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

from .view_mode import (ImageMode, CloudMode, LidarFrameVizMode,
                        SimpleMode, ReflMode, RGBMode,
                        HDRRGBMode,
                        NormalsMode, RingMode, TimestampMode,
                        is_norm_reflectivity_mode, CloudPaletteItem, SensorMode)
from ouster.sdk._bindings.viz import (Cloud, Image, Label, PointViz, Mesh, Cuboid,
                   ObjectOverlay,
                   calref_palette, spezia_palette, spezia_cal_ref_palette, grey_palette,
                   grey_cal_ref_palette, viridis_palette, viridis_cal_ref_palette,
                   magma_palette, magma_cal_ref_palette, MouseButton, MouseButtonEvent,
                   EventModifierKeys, WindowCtx)
import ouster.sdk.core as core
from ouster.sdk.core import (ChanField, FrameSet)
from ouster.sdk.util import img_aspect_ratio
from .widgets import ToggleImage, ToggleLabel, ToggleMesh
from ouster.sdk.core import ZONE_OCCUPANCY_FIELDNAME, ZONE_STATES_FIELDNAME, \
    CoordinateFrame, ClassMapSet, first_valid_column_pose
from ouster.sdk.viz.mesh import clouds_from_zrb, voxel_style_mesh_from_zrb
from ouster.sdk._bindings.viz import precompute_voxel_vertices
from ouster.sdk._deprecation import deprecated_alias
from enum import Enum


class ZoneSelectionMode(Enum):
    NONE = 0
    LIVE = 1
    ALL = 2


class ZoneRenderMode(Enum):
    STL = 0
    CLOUDS = 1
    STL_AND_CLOUDS = 2
    VOXEL_MESH = 3


class ObjectColorMode(Enum):
    OBJECT_ID = 0
    OBJECT_CLASS = 1


class ObjectViewMode(Enum):
    OVERLAY_AND_MAIN_VIEW = 0
    OVERLAY_ONLY = 1
    MAIN_VIEW_ONLY = 2
    NONE = 3


def _flatten(xss: List[List[Any]]) -> List[Any]:
    """Flattens a list of lists."""
    return [x for xs in xss for x in xs]


def _sync_object_pose_from_frame(obj: core.Object, frame: core.LidarFrame) -> None:
    """Update an object's body-to-world pose from the frame at its timestamp."""
    if obj.timestamp == 0:
        return
    try:
        obj.body_to_world = core.pose_at_timestamp(frame, obj.timestamp)
    except ValueError:
        pass


def _object_for_overlay_cuboid(obj: core.Object, frame: core.LidarFrame) -> core.Object:
    """Return an object whose pose is dewarped for the 2D panorama overlay."""
    if obj.timestamp == 0:
        return obj

    pose_ref = core.Pose(first_valid_column_pose(frame))
    overlay_obj = copy.copy(obj)
    overlay_obj.body_to_world = pose_ref
    return overlay_obj


@dataclass
class ImgModeItem:
    """Image mode for specific return with explicit name."""
    mode: ImageMode
    name: str
    return_num: int = 0


@dataclass
class VizExtraMode:
    """Image/Cloud mode factory func

    Used to embed viz modes from external plugins.
    """
    func: Callable[[], LidarFrameVizMode]

    def create(self,
               info: Optional[core.SensorInfo] = None) -> LidarFrameVizMode:
        extra_mode = self.func()
        if info and hasattr(extra_mode, "_info") and extra_mode._info is None:
            extra_mode._info = info
        return extra_mode


# Viz modes added externally
_viz_extra_modes: List[VizExtraMode]
_viz_extra_modes = []

# Viz palettes added externally
_viz_extra_palettes: List[CloudPaletteItem]
_viz_extra_palettes = []


def _hsv_to_rgb(H, S, V):
    ''' Converts an integer HSV tuple (value range from 0 to 255) to an RGB tuple '''
    # Check if the color is Grayscale
    if S == 0:
        return (V, V, V)

    # Make hue 0-5
    region = H // 43

    # Find remainder part, make it from 0-255
    remainder = (H - (region * 43)) * 6

    # Calculate temp vars, doing integer multiplication
    P = (V * (255 - S)) >> 8
    Q = (V * (255 - ((S * remainder) >> 8))) >> 8
    T = (V * (255 - ((S * (255 - remainder)) >> 8))) >> 8

    # Assign temp vars based on color cone region
    if region == 0:
        return (V, T, P)
    elif region == 1:
        return (Q, V, P)
    elif region == 2:
        return (P, V, T)
    elif region == 3:
        return (P, Q, V)
    elif region == 4:
        return (T, P, V)
    else:
        return (V, P, Q)


# Generate a rainbow palette
rainbow_palette = np.zeros((256, 3), np.float32)
for i in range(0, 256):
    res = _hsv_to_rgb(int(i * 230 / 255), 255, 255)
    rainbow_palette[i, 0] = res[0] / 255
    rainbow_palette[i, 1] = res[1] / 255
    rainbow_palette[i, 2] = res[2] / 255


discrete_rainbow_palette = rainbow_palette[::len(rainbow_palette) // 36]


class Palettes:
    """Represents the color palettes used within an instance of LidarFrameViz.
    Also keeps track of the palette currently in use."""

    def __init__(self, _ext_palettes: List[CloudPaletteItem]):
        """Initialize a Palettes object, which populates two lists
        of palettes - one for normal view modes and one for ReflMode."""

        # Note these 2 palette arrays must always be the same length
        self._cloud_palettes: List[CloudPaletteItem]
        self._cloud_palettes = [
            CloudPaletteItem("Cal. Ref", calref_palette),
            CloudPaletteItem("Ouster Colors", spezia_palette),
            CloudPaletteItem("Greyscale", grey_palette),
            CloudPaletteItem("Viridis", viridis_palette),
            CloudPaletteItem("Magma", magma_palette),
            CloudPaletteItem("Rainbow", rainbow_palette)
        ]

        self._refl_cloud_palettes: List[CloudPaletteItem]
        self._refl_cloud_palettes = [
            CloudPaletteItem("Cal. Ref", calref_palette),
            CloudPaletteItem("Cal. Ref. Ouster Colors",
                             spezia_cal_ref_palette),
            CloudPaletteItem("Cal. Ref. Greyscale", grey_cal_ref_palette),
            CloudPaletteItem("Cal. Ref. Viridis", viridis_cal_ref_palette),
            CloudPaletteItem("Cal. Ref. Magma", magma_cal_ref_palette),
            CloudPaletteItem("Rainbow", rainbow_palette)
        ]

        # Add extra color palettes, usually inserted through plugins
        self._cloud_palettes.extend(_viz_extra_palettes)
        self._refl_cloud_palettes.extend(_viz_extra_palettes)

        self._cloud_palettes.extend(_ext_palettes or [])
        self._refl_cloud_palettes.extend(_ext_palettes or [])

        # the index of the current palette used for the clouds
        self._cloud_palette_ind = 0

        assert len(self._cloud_palettes) == len(self._refl_cloud_palettes)

    def set_palette(self, index: int) -> None:
        """Set the current palette index."""
        assert len(self._cloud_palettes) == len(self._refl_cloud_palettes)
        assert self._cloud_palette_ind >= 0 and self._cloud_palette_ind < len(self._cloud_palettes)
        self._cloud_palette_ind = index

    def cycle_cloud_palette(self, direction: int) -> None:
        """Updates the current palette to use."""
        assert len(self._cloud_palettes) == len(self._refl_cloud_palettes)
        self.set_palette((self._cloud_palette_ind + direction) % len(self._cloud_palettes))

    def get_palette(self, cloud_mode: CloudMode) -> CloudPaletteItem:
        """Gets the current color palette depending on the view mode."""
        assert len(self._cloud_palettes) == len(self._refl_cloud_palettes)
        refl_mode = is_norm_reflectivity_mode(cloud_mode)
        if refl_mode:
            return self._refl_cloud_palettes[self._cloud_palette_ind]
        else:
            return self._cloud_palettes[self._cloud_palette_ind]

    def get_palette_by_name(self, name: str) -> CloudPaletteItem:
        for palette in self._cloud_palettes:
            if palette.name == name:
                return palette
        raise KeyError(name)

    def get_current_palette(self) -> CloudPaletteItem:
        """Gets the name of the current palette."""
        return self._cloud_palettes[self._cloud_palette_ind]


def triggered_live_zone_color(palette, zone_id):
    return (*palette[(int(zone_id) * 20) % palette.shape[0]], 1)


class Selection2d:
    def __init__(self, p1: Tuple[int, int],
        p2: Tuple[int, int],
        sensor_index: int,
        sensor: 'SensorModel',
        image_index: int,
        image: Image):

        self._p1 = p1
        self._p2 = p2
        self._sensor_index = sensor_index
        self._sensor = sensor
        self._image_index = image_index
        self._image = image
        self._update_mask()
        self._finalized = False

    @property
    def sensor(self):
        return self._sensor

    @property
    def finalized(self):
        return self._finalized

    def finalize(self):
        self._finalized = True

    def __str__(self):
        return f'2d selection {self._p1} - {self._p2}'

    @property
    def p1(self):
        return self._p1

    @p1.setter
    def p1(self, p1):
        self._p1 = p1
        self._update_mask()

    @property
    def p2(self):
        return self._p2

    @p2.setter
    def p2(self, p2):
        self._p2 = p2
        self._update_mask()

    def _update_mask(self):
        min_x = min(self.p1[0], self.p2[0])
        max_x = max(self.p1[0], self.p2[0])
        min_y = min(self.p1[1], self.p2[1])
        max_y = max(self.p1[1], self.p2[1])
        self._aoi_mask = np.zeros((self._sensor._meta.h, self._sensor._meta.w), np.float32)
        self._aoi_mask[min_x:max_x, min_y:max_y] = 1

    @property
    def area(self) -> int:
        """
        Calculates and returns the area of the rectangular selection.

        The area is calculated as the absolute difference between the
        x and y coordinates, ensuring a positive result regardless of the
        points' order.
        """
        x1, y1 = self._p1
        x2, y2 = self._p2

        width = abs(x2 - x1)
        height = abs(y2 - y1)

        return width * height


_colors = []
_colors.append((255, 0, 0))
_colors.append((255, 255, 0))
_colors.append((255, 0, 255))
_colors.append((0, 255, 255))
_colors.append((0, 0, 255))
_colors.append((0, 255, 0))
_colors.append((255, 255, 255))
_colors.append((255, 191, 0))
_colors.append((0, 120, 0))


class SensorModel:
    """
    A model object representing viz state for a single sensor.
    """

    def __init__(self, viz: PointViz, meta, *, _img_aspect_ratio: float = 0,
                 palettes: Palettes = Palettes([]), index: Optional[int] = None):
        self._viz = viz
        self._enabled = True
        self._meta = meta
        self._xyzlut = core.XYZLut(meta, use_extrinsics=True)
        self._palettes = palettes
        self._zone_palette = palettes.get_palette_by_name('Rainbow').palette

        self._num_clouds = 2
        self._clouds: List[Cloud] = []
        self._cloud_masks: List[np.ndarray] = []
        for i in range(self._num_clouds):
            self._clouds.append(Cloud(self._xyzlut))
            mask = np.zeros((meta.h, meta.w, 4), dtype=np.uint8)
            mask[:, :, 1] = 255
            self._cloud_masks.append(mask)
        self._cloud_modes: Dict[str, CloudMode] = {}

        self._selected_pts: Dict[int, np.ndarray] = {}
        self._selecting_pts: Dict[int, np.ndarray] = {}

        for idx, cloud in enumerate(self._clouds):
            def cb(ret_idx, selected_pts, pts):
                # update our list of selected points
                # note, be careful to not capture the viz (even indirectly) here
                # or it creates a weird reference leak
                selected_pts[ret_idx] = pts

            cloud.set_on_select(partial(cb, idx, self._selecting_pts))

        self._palette_dirty = [True] * len(self._clouds)

        self._num_images = 2
        self._images: List[Image] = []
        self._image_modes: Dict[str, ImgModeItem] = {}
        for i in range(self._num_images):
            self._images.append(Image())

        self._object_overlay = ObjectOverlay()
        self._object_overlay.set_sensor_info(self._meta)

        if _img_aspect_ratio:
            self._img_aspect_ratio = _img_aspect_ratio
        else:
            self._img_aspect_ratio = img_aspect_ratio(meta)

        self._modes: List[LidarFrameVizMode] = []

        # Add extra viz mode, usually inserted through plugins
        self._modes.extend([vm.create(meta) for vm in _viz_extra_modes])
        self._modes.append(RingMode(meta))
        self._modes.append(TimestampMode(meta))
        if index is not None:
            self._color = _colors[index % len(_colors)]
            self._modes.append(SensorMode(meta, self._color))

        # TODO[tws] decide whether it's necessary to provide extra modes via the constructor
        # self._modes.extend(_ext_modes or [])

        self._black_palette = np.zeros((256, 3), dtype=np.float32)

        self._populate_image_cloud_modes()
        self._image_calref_palette = copy.deepcopy(calref_palette)
        self._image_calref_palette[0] = [0.1, 0.1, 0.1]

        self._show_sky = False

        self._has_imu_data = False
        self._imu_plot = ToggleImage(viz, initially_visible=True)
        self._sensor_label = ToggleLabel(viz, "", (0, 1), initially_visible=True)  # for the OSD

        self._stl_meshes = {}
        self._stl_mesh_labels = {}
        self._zrb_clouds = {}
        self._zrb_meshes: Dict[int, Optional[ToggleMesh]] = {}
        self._zone_states: Optional[np.ndarray] = None
        if meta.format.zone_monitoring_enabled and meta.zone_set is not None:
            zone_set = meta.zone_set
            self._voxel_vertices = precompute_voxel_vertices(self._meta)
            for zone_id, zone in zone_set.zones.items():
                # TODO[tws] C++
                if zone.stl is not None:
                    simple_mesh = zone.stl.to_mesh()
                    if len(simple_mesh.triangles) < 1:
                        print(f"Warning: mesh for zone {zone_id} is invalid.")
                        continue
                    mesh = Mesh.from_simple_mesh(simple_mesh)
                    sensor_to_body_transform = zone_set.sensor_to_body_transform \
                        if zone.stl.coordinate_frame == CoordinateFrame.BODY else np.eye(4)
                    zone_transform = meta.sensor_to_body @ np.linalg.inv(sensor_to_body_transform)
                    mesh.set_transform(zone_transform)
                    mesh_widget = ToggleMesh(viz, mesh, initially_visible=True)
                    self._stl_meshes[zone_id] = mesh_widget

                    # TODO[tws] bind a method for viz::Mesh to return points, or perhaps the centroid
                    mesh_label_pos = (
                        # TODO[tws] dedupe
                        simple_mesh.triangles[0].coords[0][0],
                        simple_mesh.triangles[0].coords[0][1],
                        simple_mesh.triangles[0].coords[0][2],
                        1
                    )
                    mesh_label_pos = zone_transform.dot(mesh_label_pos)
                    mesh_label = ToggleLabel(viz, '', mesh_label_pos[0:3], initially_visible=False)
                    mesh_label.scale = 0.20
                    self._stl_mesh_labels[zone_id] = mesh_label
                if zone.zrb is not None:
                    self._zrb_clouds[zone_id] = clouds_from_zrb(viz, zone.zrb, self._xyzlut)
                    self._zrb_meshes[zone_id] = None  # created on demand

    def _clear_aois(self) -> None:
        self._selected_pts.clear()

    def _populate_image_cloud_modes(self) -> None:
        # TODO[tws] de-duplicate with logic in _amend_view_modes
        self._image_modes = {name:
            ImgModeItem(mode, name, num) for mode in self._modes
            if isinstance(mode, ImageMode)
            for num, name in enumerate(mode.names)
        }

        self._cloud_modes = {m.name:
            m for m in self._modes if isinstance(m, CloudMode)}

    @staticmethod
    def _get_field_class_for_field_name(field_name, frame) -> Optional[core.FieldType]:
        for field_type in frame.field_types:
            if field_type.name == field_name:
                return field_type.field_class
        return None

    def _create_view_mode_for_field(self, field_name, frame) -> Optional[LidarFrameVizMode]:
        """Create the appropriate view mode depending on a field's name and dimensions."""
        mode: Optional[LidarFrameVizMode] = None
        if field_name in [ChanField.FLAGS2, ChanField.RANGE2, ChanField.REFLECTIVITY2,
                          ChanField.SIGNAL2, ChanField.GROUND2]:
            return None

        if field_name in frame.fields:
            if SensorModel._get_field_class_for_field_name(field_name, frame) != core.FieldClass.PIXEL_FIELD:
                return None

            field = frame.field(field_name)
            f_shape = field.shape
            if len(f_shape) == 3 and f_shape[2] == 3:   # 3D field
                upper_name = field_name.upper()
                if upper_name.startswith("NORMALS"):
                    mode = NormalsMode(field_name, info=self._meta)
                elif ("RGB" in upper_name) and field.dtype == np.float16:
                    mode = HDRRGBMode(field_name, info=self._meta)
                else:
                    mode = RGBMode(field_name, info=self._meta)
            elif len(f_shape) == 2:
                if field_name == ChanField.REFLECTIVITY:
                    mode = ReflMode(info=self._meta)
                elif field_name == ChanField.NEAR_IR:
                    mode = SimpleMode(field_name, info=self._meta, use_ae=True, use_buc=True)
                elif field_name == ChanField.WINDOW:
                    mode = SimpleMode(field_name, info=self._meta, scale=1.0 / 255.0, use_ae=False, use_buc=False)
                else:
                    mode = SimpleMode(field_name, info=self._meta, use_ae=True, use_buc=False)
        return mode

    # TODO[tws] rename
    def _amend_view_modes(self, frame):
        """
        Update the available fields for this model given a frame.
        """
        if not self._has_imu_data:
            # TODO[tws] set based on imu visualization config maybe?
            if frame.has_field('IMU_GYRO') or frame.has_field('IMU_ACC'):
                self._has_imu_data = True

        for field_name in frame.fields:
            # skip if view mode already exists
            if field_name in self._image_modes or \
               field_name == ChanField.FLAGS or \
               field_name == ChanField.FLAGS2:
                continue
            mode = self._create_view_mode_for_field(field_name, frame)
            if isinstance(mode, ImageMode):
                for num, name in enumerate(mode.names):
                    self._image_modes[name] = ImgModeItem(mode, name, num)
            if isinstance(mode, CloudMode):
                self._cloud_modes[field_name] = mode

    def update_cloud_palettes(self, cloud_mode_name) -> None:
        """Sets each cloud palette given the cloud mode name.
        Has no effect if the cloud mode is not available for this sensor."""
        cloud_mode = self._cloud_modes.get(cloud_mode_name)
        if cloud_mode:
            palette = self._palettes.get_palette(cloud_mode)
            for cloud in self._clouds:
                cloud.set_palette(palette.palette)
        else:
            # flag to set this later if we failed
            self._palette_dirty = [True] * len(self._clouds)

    _max_range: int = 100000

    def update_cloud(self, cloud, cloud_mode, range_field: str, return_num: int,
                     frame: Optional[core.LidarFrame]) -> None:
        """Updates the given Cloud with the given CloudMode."""
        if frame is None:
            # make the frame invisible if it isnt present
            cloud.set_range(np.zeros((cloud.cols, cloud.size // cloud.cols), dtype=np.uint32))
            return

        if range_field in frame.fields:
            range_data = frame.field(range_field)
        else:
            range_data = np.zeros((frame.h, frame.w), dtype=np.uint32)

        cloud.set_column_poses(frame.body_to_world.astype(np.float32))

        if return_num == 0 and self._show_sky:
            self._max_range = max(self._max_range, np.max(range_data) + 10000)
            copy = np.copy(range_data)
            copy[range_data == 0] = self._max_range
            cloud.set_range(copy)
        else:
            cloud.set_range(range_data)

        # display the cloud as black if we are missing this chanfield
        if not cloud_mode:
            cloud.set_palette(self._black_palette)
            return

        # set palette if dirty and we have the mode
        if self._palette_dirty[return_num]:
            palette = self._palettes.get_palette(cloud_mode)
            cloud.set_palette(palette.palette)
            self._palette_dirty[return_num] = False

        if cloud_mode.enabled(frame, return_num):
            cloud_mode.set_cloud_color(cloud, frame, return_num=return_num)
        else:
            cloud_mode.set_cloud_color(cloud, frame, return_num=0)

    def update_clouds(self, cloud_mode_name: str, frame: Optional[core.LidarFrame]) -> None:
        """Update range and mode for each cloud given a mode name and a frame."""
        cloud_mode = self._cloud_modes.get(cloud_mode_name, None)
        for return_num, range_field in ((0, ChanField.RANGE), (1, ChanField.RANGE2)):
            if return_num < len(self._clouds):
                self.update_cloud(self._clouds[return_num], cloud_mode, range_field, return_num, frame)

    def update_image(self, image, image_mode_item: ImgModeItem, frame: Optional[core.LidarFrame]) -> None:
        """Update the view mode of the given image."""
        # TODO[tws] optimize, move to image model(?)

        if not frame:
            image.set_image(np.zeros((self._meta.h, self._meta.w), dtype=np.float32))
            return

        refl_mode = False
        mode = image_mode_item.mode
        if mode:
            refl_mode = is_norm_reflectivity_mode(mode)
            if refl_mode:
                image.set_palette(self._image_calref_palette)
        if not refl_mode:
            image.clear_palette()

        if mode is not None and mode.enabled(frame, image_mode_item.return_num):
            mode.set_image(image, frame, image_mode_item.return_num)
        else:
            # TODO[tws]: deduplicate, e.g. by making this a method in an image model class
            image.set_image(np.zeros((self._meta.h, self._meta.w), dtype=np.float32))

    def update_images(self, image_mode_names: List[str], frame: Optional[core.LidarFrame]) -> None:
        """Update image values and mode given mode names and a frame."""
        assert len(image_mode_names) == len(self._images)
        for i in range(len(self._images)):
            image_mode_item = self._image_modes.get(image_mode_names[i])
            if image_mode_item:
                self.update_image(self._images[i], image_mode_item, frame)
            else:
                # TODO[tws]: deduplicate, e.g. by making this a method in an image model class
                self._images[i].set_image(np.zeros((self._meta.h, self._meta.w), dtype=np.float32))

    def update_zones(self, frame: Optional[core.LidarFrame] = None) -> None:
        if self._meta.zone_set is None:
            return

        # TODO[tws] use >= max zone id constant here
        live_zone_ids: Set[int] = set()
        if self._zone_states is not None:
            live_zone_ids = {zone_state.id for zone_state in self._zone_states if zone_state.id < 255}

        zrb_pose: Optional[np.ndarray] = None
        stl_pose: Optional[np.ndarray] = None
        if frame is not None:
            try:
                pose = frame.body_to_world[frame.get_last_valid_column()]
            except RuntimeError:
                pose = None
            if pose is not None:
                zrb_pose = pose
                zrb_mesh_pose = (pose @ self._meta.sensor_to_body)
                stl_pose = ((pose @ self._meta.sensor_to_body) @
                    np.linalg.inv(self._meta.zone_set.sensor_to_body_transform))

        # set mesh visibility and color
        for zone_id in self._stl_meshes.keys():
            stl_mesh = self._stl_meshes.get(zone_id)
            if stl_mesh:
                visible = False
                if self._zone_render_mode in (ZoneRenderMode.STL_AND_CLOUDS, ZoneRenderMode.STL) and self._enabled:
                    if self._zone_selection_mode == ZoneSelectionMode.ALL:
                        visible = True
                    elif self._zone_selection_mode == ZoneSelectionMode.LIVE and zone_id in live_zone_ids:
                        visible = True
                stl_mesh.visible = visible
                stl_mesh_label = self._stl_mesh_labels[zone_id]
                stl_mesh_label.hide()
                if zone_id not in live_zone_ids:
                    stl_mesh._item.set_edge_rgba((0.5, 0.5, 0.5, 0.5))
                else:
                    stl_mesh._item.set_edge_rgba((0.8, 0.8, 0.8, 0.8))

                if stl_pose is not None:
                    zone = self._meta.zone_set.zones[zone_id]
                    if zone.stl.coordinate_frame == CoordinateFrame.BODY:
                        stl_mesh._item.set_transform(stl_pose)
                    else:
                        stl_mesh._item.set_transform(zrb_mesh_pose)

        # set voxel mesh visibility
        for zone_id, zrb_mesh in self._zrb_meshes.items():
            visible = False
            if self._zone_render_mode == ZoneRenderMode.VOXEL_MESH and self._enabled:
                if self._zone_selection_mode == ZoneSelectionMode.ALL:
                    visible = True
                elif self._zone_selection_mode == ZoneSelectionMode.LIVE and zone_id in live_zone_ids:
                    visible = True
            zone = self._meta.zone_set.zones[zone_id]
            if zrb_mesh is None and zone.zrb is not None:
                zrb_mesh = ToggleMesh(
                    self._viz,
                    voxel_style_mesh_from_zrb(
                        zone.zrb,
                        self._meta,
                        self._voxel_vertices,
                        False,
                        True
                    )
                )
                zrb_mesh._item.set_transform(self._meta.sensor_to_body)
                self._zrb_meshes[zone_id] = zrb_mesh
            if zrb_mesh is not None:
                zrb_mesh.visible = visible
                if zrb_pose is not None:
                    zrb_mesh._item.set_transform(zrb_mesh_pose)

        # set zone clouds visibility
        for zone_id, (near_cloud, far_cloud) in self._zrb_clouds.items():
            visible = False
            if self._zone_render_mode in (ZoneRenderMode.STL_AND_CLOUDS, ZoneRenderMode.CLOUDS) and self._enabled:
                if self._zone_selection_mode == ZoneSelectionMode.ALL:
                    visible = True
                elif self._zone_selection_mode == ZoneSelectionMode.LIVE and zone_id in live_zone_ids:
                    visible = True
            # TODO[tws] set cloud color
            near_cloud.visible = visible
            far_cloud.visible = visible
            if zrb_pose is not None:
                near_cloud._item.set_pose(zrb_pose)
                far_cloud._item.set_pose(zrb_pose)

        # set mesh label visibility
        for zone_id, stl_mesh_label in self._stl_mesh_labels.items():
            visible = False
            if self._zone_selection_mode != ZoneSelectionMode.NONE and zone_id in live_zone_ids and self._enabled:
                visible = True
            stl_mesh_label.visible = visible

        # set triggered mesh color and label
        if self._zone_states is not None:
            for zone_state in self._zone_states:
                zone_id = zone_state.id
                live = bool(zone_state.live)
                trigger_status = zone_state.trigger_status
                stl_mesh = self._stl_meshes.get(zone_id)
                # TODO[tws] configurable colors
                live_color = triggered_live_zone_color(self._zone_palette, zone_id)
                if stl_mesh:
                    stl_mesh_label = self._stl_mesh_labels[zone_id]
                    if live:
                        stl_mesh_label.text = str(zone_state.count)
                        stl_mesh_label.rgba = live_color
                        if trigger_status:
                            stl_mesh._item.set_edge_rgba(live_color)
                    else:
                        stl_mesh_label.text = ""

    def update_zone_occupancy_cloud_and_image_masks(self, frame: Optional[core.LidarFrame], palettes: Palettes):
        if not frame:
            # TODO[tws] deactivate zones / clear masks
            return

        destaggered_mask = mask = np.zeros((self._meta.h, self._meta.w, 4), dtype=np.uint8)
        if frame.has_field(ZONE_OCCUPANCY_FIELDNAME):
            occupancy = frame.field(ZONE_OCCUPANCY_FIELDNAME)
            # TODO[tws] consider flags mode.
            # TODO[tws] color mask depending on zone color?
            if frame.has_field(ZONE_STATES_FIELDNAME):
                zone_states = frame.field(ZONE_STATES_FIELDNAME)
                for z_idx in range(16):  # TODO[tws] max live zones constant
                    zone_state = zone_states[z_idx]
                    zone_id = zone_state.id
                    # TODO[tws] use >= max zone id constant here
                    if zone_id == 255:
                        continue
                    bit = 0x1 << z_idx
                    color = triggered_live_zone_color(self._zone_palette, zone_id)
                    mask[(occupancy & bit) > 0] = np.array(color) * 255
            else:
                for z_idx in range(16):  # TODO[tws] max live zones constant
                    bit = 0x1 << z_idx
                    color = triggered_live_zone_color(self._zone_palette, z_idx)
                    mask[(occupancy & bit) > 0] = np.array(color) * 255
            destaggered_mask = core.destagger(self._meta, mask)
        self._cloud_masks[0] = mask
        for image in self._images:
            image.set_mask(destaggered_mask)

    def update_object_id_cloud_and_image_masks(self, frame: Optional[core.LidarFrame], mask_palette):
        if not frame:
            return

        for field_name in frame.fields:
            if field_name.endswith("INSTANCE_ID"):
                mask = np.zeros((self._meta.h, self._meta.w, 4), dtype=np.uint8)
                oid = frame.field(field_name)
                has_object = oid != -1
                mask[has_object] = np.dstack(
                    (
                        mask_palette[oid % len(mask_palette)] * 255.0,
                        np.ones((frame.h, frame.w)) * 255.0
                    )
                )[has_object]
                destaggered_mask = core.destagger(self._meta, mask)
                self._cloud_masks[0] = mask
                for image in self._images:
                    image.set_mask(destaggered_mask)

    def hide_all_zone_geometry(self):
        for near_cloud, far_cloud in self._zrb_clouds.values():
            near_cloud.visible = False
            far_cloud.visible = False
        for stl_mesh in self._stl_meshes.values():
            stl_mesh.visible = False
        for stl_mesh_label in self._stl_mesh_labels.values():
            stl_mesh_label.hide()
        for zrb_mesh in self._zrb_meshes.values():
            if zrb_mesh is not None:
                zrb_mesh.visible = False

    def set_zone_selection_and_render_mode(self, selection_mode: ZoneSelectionMode, render_mode: ZoneRenderMode):
        self.hide_all_zone_geometry()
        self._zone_selection_mode = selection_mode
        self._zone_render_mode = render_mode
        self.update_zones()


class LidarFrameVizModel:
    def __init__(self, viz: PointViz, metas: List[core.SensorInfo], *, _img_aspect_ratio: float):
        self._viz = viz
        self._cloud_pt_size: float = 2.0
        self._metas = metas
        assert len(metas) > 0, "ERROR: Expecting at least one sensor"
        _ext_palettes: List[CloudPaletteItem] = []   # TODO[tws] implement
        self._palettes: Palettes = Palettes(_ext_palettes)
        self._sensors: List[SensorModel] = []
        for idx, meta in enumerate(self._metas):
            # only set index (which enables sensor based coloring) if we have > 1 sensor
            model = SensorModel(viz, meta, _img_aspect_ratio=_img_aspect_ratio,
                                palettes=self._palettes,
                                index=None if len(self._metas) == 1 else idx)
            self._sensors.append(model)
        self._max_clouds = max(sensor._num_clouds for sensor in self._sensors)
        self._max_images = max(sensor._num_images for sensor in self._sensors)
        self._cloud_mode_ind: int = 0
        self._cloud_mode_name: str = ''
        self._cloud_enabled = [True]
        self._image_mode_ind = [0] * self._max_images
        self._image_mode_names = [''] * self._max_images
        self._set_cloud_pt_size(self._cloud_pt_size)
        self._known_fields: Set[str] = set()              # set of known fields
        self._cloud_palette_name = ''
        self._flags_palette = discrete_rainbow_palette
        self._object_color_mode = ObjectColorMode.OBJECT_ID
        self._object_view_mode = ObjectViewMode.OVERLAY_AND_MAIN_VIEW
        self._frame_set: FrameSet = FrameSet([None] * len(metas))

        # Object-related
        self._objects: Dict[core.Object, Cuboid] = {}
        self._object_labels: Dict[core.Object, ToggleLabel] = {}
        self._class_maps: ClassMapSet = ClassMapSet({})
        self._class_map_name: str = ''

        # AOI-related
        self._mouse_down_image: Optional[int] = None
        self._current_selection: List[Selection2d] = []
        self._aoi_labels = [Label("", 0, 0) for _ in range(self._max_images)]
        self._3d_aoi_label = Label("", 0, 0)
        self._preexisting_selection: bool = False

        # regrettable, since images aren't always aware of the context they're rendered in
        self._ctx: Optional[WindowCtx] = None
        self._img_size_fraction = 4

        self._flip_images = False
        self._show_one_image = False
        self._objects_selected = False

        self._move_queue: LifoQueue[Optional[Tuple]] = LifoQueue()
        self._queue_idx = 0

        # create thread to handle selection requests in parallel
        self._aoi_lock = threading.RLock()

        def worker(mq, this):
            last_index = -1
            while True:
                item = mq.get()
                if item is None:
                    break

                self = this()
                idx, sx, sy, ex, ey, update = item
                # ignore old requests, we only want the most recent
                if idx < last_index:
                    continue

                last_index = idx
                self.clear_masks()
                self._viz.pick(sx, sy, ex, ey)
                if self._objects_selected:
                    self._viz.update()
                    self._objects_selected = False

                # swap in the new selections and then update the AoIs
                with self._aoi_lock:
                    for sensor in self._sensors:
                        sensor._selected_pts.clear()
                        for one, two in sensor._selecting_pts.items():
                            sensor._selected_pts[one] = two
                        sensor._selecting_pts.clear()

                self.update_aoi()

        self._pick_thread = threading.Thread(target=worker, daemon=True, args=(self._move_queue, weakref.ref(self)))
        self._pick_thread.start()

    def __del__(self):
        self.finish()

    def finish(self):
        """Stop any background processing"""
        self._move_queue.put(None)

    def _set_cloud_pt_size(self, point_size):
        for sensor in self._sensors:
            for cloud in sensor._clouds:
                cloud.set_point_size(point_size)

    # TODO[tws] likely remove
    @property
    def metadata(self) -> List[core.SensorInfo]:
        """Metadatas for the displayed sensors."""
        return self._metas

    def set_class_maps(self, class_maps: core.ClassMapSet) -> None:
        """Set the class maps for the visualizer."""
        self._class_maps = class_maps
        self._class_map_name = sorted(class_maps.keys())[0] if class_maps else ''

    def _amend_view_modes_all(self, frames: FrameSet) -> None:
        """Add new image and cloud view modes to each sensor for each any new fields in the received frames."""

        assert len(frames) == len(self._sensors)
        for frame, sensor in zip(frames, self._sensors):
            if not frame:
                continue
            # TODO[tws] optimize; _known_fields is only used to check for
            # whether preferred fields are available for default view modes
            self._known_fields.update(frame.fields)
            sensor._amend_view_modes(frame)

        # if no view modes are set yet, set them!
        self._use_default_view_modes()

    def update_cloud_palette_name(self):
        """Gets the name of the palette used for the point clouds."""
        # the cloud palette name *technically* may not be the same across all sensors
        # we'll find the palette name of the first sensor
        if len(self._sensors) > 0:
            cloud_mode = self._sensors[0]._cloud_modes.get(self._cloud_mode_name, None)
            palette = self._palettes.get_palette(cloud_mode)
            self._cloud_palette_name = palette.name

    def update_cloud_palettes(self):
        """Updates the point clouds to use the currently-selected palette."""
        self.update_cloud_palette_name()
        for sensor in self._sensors:
            sensor.update_cloud_palettes(self._cloud_mode_name)

    def sorted_cloud_mode_names(self):
        """Returns all the cloud mode names."""
        return sorted(
            list(
                set(
                    _flatten([sensor._cloud_modes.keys() for sensor in self._sensors])
                )
            ),
            key = lambda key: key.lower()
        )

    def sorted_image_mode_names(self):
        """Returns the image mode names, limited to those for fields we've seen in at least one LidarFrame."""
        return sorted(
            list(
                set(
                    _flatten([sensor._image_modes.keys() for sensor in self._sensors])
                ) & self._known_fields  # limit to fields seen in LidarFrames
            ),
            key = lambda key: key.lower()
        )

    def select_cloud_mode(self, name: str) -> bool:
        """Updates the currently selected cloud mode from the list of all available cloud modes.
           Returns false if the requested mode is not available.
        """
        all_cloud_mode_names = self.sorted_cloud_mode_names()
        if name not in all_cloud_mode_names:
            return False

        self._cloud_mode_ind = all_cloud_mode_names.index(name)
        self._cloud_mode_name = name
        self.update_cloud_palettes()
        return True

    def cycle_cloud_mode(self, direction: int):
        """Updates the currently selected cloud mode from the list of all available cloud modes."""
        all_cloud_mode_names = self.sorted_cloud_mode_names()
        self._cloud_mode_ind = (self._cloud_mode_ind + direction) % len(all_cloud_mode_names)
        self._cloud_mode_name = all_cloud_mode_names[self._cloud_mode_ind]
        self.update_cloud_palettes()

    def select_image_mode(self, i: int, name: str) -> bool:
        """Updates the currently selected image mode from the list of all available cloud modes.
           Returns false if the requested mode is not available.
        """
        all_image_mode_names = self.sorted_image_mode_names()
        if name not in all_image_mode_names:
            return False

        self._image_mode_ind[i] = all_image_mode_names.index(name)
        self._image_mode_names[i] = name
        return True

    def cycle_image_mode(self, i: int, direction: int):
        """Updates the currently selected image mode from the list of all available cloud modes."""
        all_image_mode_names = self.sorted_image_mode_names()
        if len(all_image_mode_names) == 0:
            return
        self._image_mode_ind[i] = (self._image_mode_ind[i] + direction) % len(all_image_mode_names)
        self._image_mode_names[i] = all_image_mode_names[self._image_mode_ind[i]]
        # TODO[tws] optimize update image palettes

    def _use_default_view_modes(self):
        """Try to set the image and cloud view modes to sensible defaults, depending on what
        fields have been discovered via _amend_view_modes_all.

        If no sensible default can be found, the method will
        pick the view modes associated with the first fields.
        """
        if self._cloud_mode_name and self._image_mode_names:
            # exit early if the view modes are already set
            return

        sorted_cloud_mode_names = self.sorted_cloud_mode_names()
        sorted_image_mode_names = self.sorted_image_mode_names()
        if not sorted_image_mode_names or not sorted_cloud_mode_names:
            return

        # Set the modes for image 1 and cloud
        if "RGB" in sorted_image_mode_names:
            # Always show RGB if present in image 1 and cloud
            self._image_mode_ind[1] = sorted_image_mode_names.index("RGB")
            self._cloud_mode_ind = sorted_cloud_mode_names.index("RGB")
        else:
            # Otherwise prefer REFLECTIVITY2 or NEAR_IR for image 1
            # For cloud prefer REFLECTIVITY
            preferred_field = ChanField.REFLECTIVITY2
            if ChanField.REFLECTIVITY2 not in self._known_fields:
                preferred_field = ChanField.NEAR_IR
            try:
                self._image_mode_ind[1] = sorted_image_mode_names.index(preferred_field)
                self._cloud_mode_ind = sorted_cloud_mode_names.index(ChanField.REFLECTIVITY)
            except ValueError:
                self._image_mode_ind[1] = 0
                self._cloud_mode_ind = 0

        # Finally set the mode for image 0, which should be reflectivity
        try:
            self._image_mode_ind[0] = sorted_image_mode_names.index(ChanField.REFLECTIVITY)
        except ValueError:
            self._image_mode_ind[0] = 0

        self._cloud_mode_name = sorted_cloud_mode_names[self._cloud_mode_ind]
        for i in range(2):
            self._image_mode_names[i] = sorted_image_mode_names[self._image_mode_ind[i]]
        self.update_cloud_palettes()

    def update_objects_for_list(self, key: str, object_list):
        def cuboid_select_cb(lsv_weakref, viz_weakref, cuboid_weakref, selected_color, obj, lsvm_weakref, _) -> None:
            # ignored param is a list of ids
            lsv = lsv_weakref()
            viz = viz_weakref()
            lsvm = lsvm_weakref()
            cuboid = cuboid_weakref()
            if not lsv or not viz or not cuboid or not lsvm:
                return
            if obj.id in lsv._object_labels:
                return
            cuboid.set_rgba(selected_color)
            pos = (obj.body_to_world * obj.object_to_body).position
            vel_norm = np.linalg.norm(obj.velocity)
            class_map = lsv._class_maps.get(lsv._class_map_name, None)
            class_name = class_map.get(obj.class_id, 'UNKNOWN') if class_map else 'UNKNOWN'  # type: ignore
            text = f"{class_name} {obj.id}\nvelocity: {vel_norm:.1f} m/s"
            label = ToggleLabel(viz, text, pos, initially_visible=True)
            label.scale = 1.25
            lsv._object_labels[obj.id] = label
            lsvm._objects_selected = True

        for obj in object_list:
            color_attr = obj.id if self._object_color_mode == ObjectColorMode.OBJECT_ID else obj.class_id
            color = self._flags_palette[color_attr % len(self._flags_palette)]
            color = (*self._flags_palette[color_attr % len(self._flags_palette)], 0.3)
            cuboid = Cuboid.from_object(obj, color)
            lsv_weakref = weakref.ref(self)
            viz_weakref = weakref.ref(self._viz)
            cuboid_weakref = weakref.ref(cuboid)
            selected_color = (*color[0:3], 0.9)
            cb = partial(
                cuboid_select_cb,
                lsv_weakref,
                viz_weakref,
                cuboid_weakref,
                selected_color,
                obj,
                weakref.ref(self)
            )
            cuboid.set_on_select(cb)
            if self._object_view_mode in (ObjectViewMode.OVERLAY_AND_MAIN_VIEW, ObjectViewMode.MAIN_VIEW_ONLY):
                self._viz.add(cuboid)
            self._objects[obj] = cuboid

            if obj.id in self._object_labels:
                label = self._object_labels[obj.id]
                label.visible = self._object_view_mode in (
                    ObjectViewMode.OVERLAY_AND_MAIN_VIEW,
                    ObjectViewMode.MAIN_VIEW_ONLY
                )
                label.position = (obj.body_to_world * obj.object_to_body).position
                vel_norm = np.linalg.norm(obj.velocity)
                class_map = self._class_maps.get(self._class_map_name, None)
                class_name = class_map.get(obj.class_id, 'UNKNOWN') if class_map else 'UNKNOWN'  # type: ignore
                text = f"{class_name} {obj.id}\nvelocity: {vel_norm:.1f} m/s"
                label.text = text
                label.scale = 1.25
                color = (*self._flags_palette[color_attr % len(self._flags_palette)], 0.9)
                cuboid.set_rgba(color)

    def update_object_cuboids(self) -> None:
        """Add or remove cuboids from the viz based on the current object view mode."""
        if self._object_view_mode in (ObjectViewMode.OVERLAY_AND_MAIN_VIEW, ObjectViewMode.OVERLAY_ONLY):
            for sensor_idx, sensor in enumerate(self._sensors):
                frame = self._frame_set[sensor_idx]
                overlay_cuboids = []
                if frame is not None:
                    for obj, _cuboid in self._objects.items():
                        color_attr = obj.id if self._object_color_mode == ObjectColorMode.OBJECT_ID else obj.class_id
                        color = self._flags_palette[color_attr % len(self._flags_palette)]
                        rgba = (*color, 0.9) if obj.id in self._object_labels else (*color, 0.3)
                        overlay_cuboid = Cuboid.from_object(
                            _object_for_overlay_cuboid(obj, frame), rgba)
                        overlay_cuboids.append(overlay_cuboid)
                sensor._object_overlay.set_cuboids(overlay_cuboids)
        else:
            for sensor in self._sensors:
                sensor._object_overlay.set_cuboids([])

    def update_objects(self, frames: FrameSet) -> None:
        """Update the objects in the viz from the given FrameSet."""
        self._frame_set = frames

        # remove all objects
        for cuboid in self._objects.values():
            self._viz.remove(cuboid)
        self._objects.clear()

        if isinstance(frames, FrameSet):
            for key, object_list in frames.objects.items():
                self.update_objects_for_list(key, object_list)

        for frame in frames:
            if frame is None:
                continue
            for key, object_list in frame.objects.items():
                self.update_objects_for_list(key, object_list)

        self.update_object_cuboids()

    def update(self, frames: FrameSet, new_frames: bool = True) -> None:
        """Update the LidarFrameViz state with the provided frames."""
        assert len(frames) == len(self._sensors)

        self._frame_set = frames
        self._amend_view_modes_all(frames)

        # if we have a 3d selection, update it on new frames
        if new_frames:
            for sensor in self._sensors:
                if len(sensor._selected_pts):
                    self.update_3d_aoi(True)
                    break

        self.update_objects(frames)
        for frame, sensor in zip(frames, self._sensors):
            sensor.update_clouds(self._cloud_mode_name, frame)
            sensor.update_images(self._image_mode_names, frame)

            if frame:
                # TODO [tws] use midpoint pose?
                view_mat = np.linalg.inv(frame.body_to_world[0] @ sensor._meta.sensor_to_body)
                sensor._object_overlay.set_view_matrix(view_mat)
                if frame.has_field(ZONE_STATES_FIELDNAME):
                    # a zero timestamp indicates we didnt get a packet
                    if frame.has_field(ChanField.ZONE_PACKET_TIMESTAMP) and \
                       frame.field(ChanField.ZONE_PACKET_TIMESTAMP)[0] == 0:
                        sensor._zone_states = None
                    else:
                        sensor._zone_states = frame.field(ZONE_STATES_FIELDNAME)
                sensor.update_zones(frame)

                # check if we should enable dual return mode
                if len(self._cloud_enabled) == 1:
                    if frame.has_field('RANGE2'):
                        self._cloud_enabled.append(True)

    def clear_masks(self):
        for sensor_index, sensor in enumerate(self._sensors):
            meta = sensor._meta
            mask = np.zeros((meta.h, meta.w, 4), dtype=np.uint8)
            for image in sensor._images:
                image.set_mask(mask)
            for cloud in sensor._clouds:
                cloud.set_mask(mask)

    def clear_aoi(self):
        self._current_selection = []
        [aoi_label.set_text("") for aoi_label in self._aoi_labels]
        self.clear_masks()
        self._3d_aoi_label.set_text("")
        with self._aoi_lock:
            for sensor_index, sensor in enumerate(self._sensors):
                sensor._selected_pts.clear()

    def clear_object_selection(self):
        for label in self._object_labels.values():
            label.hide()
        self._object_labels.clear()

    def image_and_pixel_for_viewport_coordinate(self, ctx, x, y):
        pixel = None
        for sensor_index, sensor in enumerate(self._sensors):
            if not sensor._enabled:
                continue
            for image_idx, image in enumerate(sensor._images):
                try:
                    pixel = image.viewport_coordinates_to_image_pixel(ctx, x, y)
                    # TODO[tws] add accessors for image width, height to PointViz
                    if pixel[0] >= 0 and pixel[0] < sensor._meta.h and pixel[1] >= 0 and pixel[1] < sensor._meta.w:
                        return sensor_index, image_idx, image, pixel
                except RuntimeError:
                    # TWS 20251119: this can happen when the image has zero data or width/height
                    # in such cases we can't compute a valid relative pixel coordinate
                    continue
        return None, None, None, None

    _pick_start = None

    def mouse_button_handler(self, ctx: WindowCtx,
        button: MouseButton, event: MouseButtonEvent, mods: EventModifierKeys) -> bool:

        ret = True
        if button == MouseButton.MOUSE_BUTTON_RIGHT and event == MouseButtonEvent.MOUSE_BUTTON_PRESSED:
            image = None
            if self._img_size_fraction > 0:
                sensor_index, image_idx, image, pixel = self.image_and_pixel_for_viewport_coordinate(
                    ctx, ctx.mouse_x, ctx.mouse_y
                )
            if image:
                self.clear_aoi()
                sensor = self._sensors[sensor_index]
                self._mouse_down_image = image_idx
                self._current_selection = [
                    Selection2d(pixel, (pixel[0] + 1, pixel[1] + 1), sensor_index, sensor, image_idx, image)
                    for image_idx, image in enumerate(sensor._images)
                ]
                ret = False
            else:
                self.clear_aoi()
                if mods & EventModifierKeys.MOD_SHIFT == 0:
                    self.clear_object_selection()
                self.update_overlay_colors(self._flags_palette)
                self._pick_start = (int(ctx.mouse_x), int(ctx.mouse_y))
                ret = False
            self.update_aoi(ctx)

        if self._current_selection and \
            button == MouseButton.MOUSE_BUTTON_RIGHT and \
            event == MouseButtonEvent.MOUSE_BUTTON_RELEASED:

            # clear selections if there was a preexisting selection and the new selection is a
            # single pixel (mouse click and release), or if there is a selection of area == 0
            area = self._current_selection[0].area
            if area <= 1 and self._preexisting_selection or area == 0:
                self.clear_aoi()
                self._preexisting_selection = False
            else:
                [selection.finalize() for selection in self._current_selection]
                self._preexisting_selection = True
            ret = False
            self.update_aoi(ctx)

        if button == MouseButton.MOUSE_BUTTON_RIGHT and \
            event == MouseButtonEvent.MOUSE_BUTTON_RELEASED:
            if self._pick_start is not None:
                x = int(ctx.mouse_x)
                y = int(ctx.mouse_y)
                x2, y2 = self._pick_start

                # enqueue the final pick request
                self._move_queue.put((self._queue_idx, x, y, x2, y2, True), block=False)
                self._queue_idx += 1

                self._pick_start = None

        return ret

    _last_selection_origin = None

    def update_3d_aoi(self, force_origin=False):
        with self._aoi_lock:
            # fill out the mask
            for sensor_idx, model in enumerate(self._sensors):
                for ret_idx, selection in model._selected_pts.items():
                    mask = np.zeros((model._meta.h, model._meta.w, 4), dtype=np.uint8)
                    mask1d = mask.reshape([-1, 4])
                    mask1d[selection] = (255, 0, 0, 153)
                    # update masks for relevant returns
                    for idx, img in enumerate(model._images):

                        img_mode = self._image_mode_names[idx]
                        if img_mode in [
                            ChanField.FLAGS2, ChanField.RANGE2, ChanField.REFLECTIVITY2, ChanField.SIGNAL2
                        ]:
                            if ret_idx == 1:
                                img.set_mask(core.destagger(model._meta, mask))
                        elif img_mode in [
                            ChanField.FLAGS, ChanField.RANGE, ChanField.REFLECTIVITY, ChanField.SIGNAL
                        ]:
                            if ret_idx == 0:
                                img.set_mask(core.destagger(model._meta, mask))
                        else:
                            # show selections in either return
                            if len(model._selected_pts) > 1:
                                img_mask = np.copy(mask)
                                img_mask1d = img_mask.reshape([-1, 4])
                                for _, mask_pts in model._selected_pts.items():
                                    img_mask1d[mask_pts] = (255, 0, 0, 153)
                            else:
                                img_mask = mask
                            img.set_mask(core.destagger(model._meta, img_mask))

                    model._clouds[ret_idx].set_mask(mask)

            num_pts = 0
            data = None
            position = None
            # check if our origin is still good so we can keep it stable
            if self._last_selection_origin:
                last_sensor, last_frame, last_pt = self._last_selection_origin
                selection = self._sensors[last_sensor]._selected_pts.get(last_frame)
                if (selection is None or last_pt not in selection) or force_origin:
                    self._last_selection_origin = None

            # accumulate data points in the selection for statistics
            # also find a 3D point to place the label
            point = None
            for sensor_idx, model in enumerate(self._sensors):
                for idx, selection in model._selected_pts.items():
                    frame = self._frame_set[sensor_idx]
                    if frame is None:
                        continue
                    if len(selection) == 0:
                        continue

                    num_pts += len(selection)

                    first_index = int(selection[0])
                    frame_w = int(frame.w)
                    col = int(first_index % frame_w)
                    row = int(first_index // frame_w)
                    point = (sensor_idx, row, col)

                    # if we need a new origin, pick one
                    if self._last_selection_origin is None:
                        if idx == 0:
                            rng = frame.field("RANGE").reshape((-1,))[first_index]
                        else:
                            rng = frame.field("RANGE2").reshape((-1,))[first_index]
                        position = copy.copy(model._xyzlut.offset[first_index])
                        position += model._xyzlut.direction[first_index] * rng
                        position = frame.body_to_world[col] @ (position[0], position[1], position[2], 1.0)
                        self._3d_aoi_label.set_position(position[0], position[1], position[2])
                        self._last_selection_origin = (sensor_idx, idx, first_index)

                    # accumulate data points for statistics
                    if self._cloud_mode_name == "RING":
                        frame_data = selection // frame.w
                    elif self._cloud_mode_name == "TIMESTAMP":
                        frame_data = np.tile(frame.timestamp, (frame.h, 1)).reshape((-1,))[selection]
                    elif self._cloud_mode_name == "SENSOR":
                        frame_data = np.tile(sensor_idx, (frame.h * frame.w))[selection]
                    elif not frame.has_field(self._cloud_mode_name):
                        continue
                    else:
                        field = frame.field(self._cloud_mode_name)
                        if field.ndim > 2:
                            frame_data = field.reshape((-1, field.shape[-1]))[selection, :]
                        else:
                            frame_data = field.reshape((-1,))[selection]
                    if data is None:
                        data = frame_data
                    else:
                        data = np.concatenate((data, frame_data), axis = 0)

            if num_pts > 0:
                self._viz.add(self._3d_aoi_label)

                label_text = f"{self._cloud_mode_name}\n"
                if data is None:
                    label_text += (
                        "min/max:   ?  ?\n"
                        "mean/std:  ?  ?\n"
                    )
                else:
                    if data.ndim > 1:
                        def format(values: np.ndarray) -> str:
                            return "[" + ", ".join(f"{v:6.3f}" for v in values) + "]"
                    else:
                        def format(values: np.ndarray) -> str:
                            return f"{values:8.1f}"

                    # Use float32 accumulators for fp16 RGB stats; summing/squaring AOI
                    # pixels in float16 can overflow even when the source values are valid.
                    stats_dtype = (np.float32 if np.issubdtype(data.dtype, np.floating) and
                                   data.dtype.itemsize < np.dtype(np.float32).itemsize else None)
                    min = np.min(data, axis=0)
                    max = np.max(data, axis=0)
                    mean = np.mean(data, axis=0, dtype=stats_dtype)
                    std = np.std(data, axis=0, dtype=stats_dtype)
                    label_text += (
                        f"min/max:   {format(min)} {format(max)}\n"
                        f"mean/std:{format(mean)} {format(std)}\n"
                    )
                label_text += f"{num_pts} pts"
                if num_pts == 1:
                    label_text += f"\nsensor: {point[0]}\n"
                    col = point[2]
                    row = point[1]
                    sensor_info = self._sensors[point[0]]._meta
                    shift = int(sensor_info.format.pixel_shift_by_row[row])
                    w = int(sensor_info.w)
                    destaggered_col = col + shift
                    if destaggered_col >= w:
                        destaggered_col = destaggered_col - w
                    elif destaggered_col < 0:
                        destaggered_col = destaggered_col + w
                    label_text += f"[{col} {row}] (staggered)\n"
                    label_text += f"[{destaggered_col} {row}] (destaggered)"
                self._3d_aoi_label.set_text(label_text)
            else:
                self._3d_aoi_label.set_position(-10, -10, False, False)
                self._last_selection_origin = None
            self._viz.update()

    def update_aoi(self, ctx: Optional[WindowCtx] = None):
        with self._aoi_lock:
            found = False
            for sensor_idx, model in enumerate(self._sensors):
                for idx, selected in model._selected_pts.items():
                    if len(selected) > 0:
                        found = True

            if found:
                # update 3D AOI
                self.update_3d_aoi()
                return

            # try and update masks from sensors
            for sensor in self._sensors:
                sensor._clear_aois()
            self._3d_aoi_label.set_position(-10, -10, False, True)

            if not self._current_selection:
                return
            if ctx:
                self._ctx = ctx

            # clear the masks - important because the image mode may have changed
            self.clear_masks()

            for selection in self._current_selection:
                image_idx = selection._image_index
                sensor = selection._sensor
                image = sensor._images[image_idx]
                meta = sensor._meta
                p1 = selection.p1
                p2 = selection.p2

                # compute the image mask given the selection area
                mask = np.zeros((meta.h, meta.w, 4), dtype=np.uint8)
                min_x = min(p1[0], p2[0])
                max_x = max(p1[0], p2[0])
                min_y = min(p1[1], p2[1])
                max_y = max(p1[1], p2[1])
                mask[min_x:max_x, min_y:max_y, :] = (255, 0, 0, 153)
                image.set_mask(mask)

                # compute the cloud mask
                cloud_mask = np.zeros((meta.h, meta.w, 4), dtype=np.uint8)
                cloud_mask[min_x:max_x, min_y:max_y, :] = (255, 0, 0, 204)
                cloud_mask = core.destagger(meta, cloud_mask, inverse=True)

                # mask the first return cloud unless the AOI is on a 2nd return field
                key = self._image_mode_names[selection._image_index]
                cloud_index = 1 if key in [
                    ChanField.FLAGS2, ChanField.RANGE2, ChanField.REFLECTIVITY2, ChanField.SIGNAL2
                ] else 0
                sensor._clouds[cloud_index].set_mask(cloud_mask)

                # set the location of the text
                assert self._ctx
                mask_upper_right_viewport_coordinates = image.image_pixel_to_viewport_coordinates(
                    self._ctx, (max_x, max_y)
                )

                aoi_label_x = (mask_upper_right_viewport_coordinates[0] + 12) / self._ctx.viewport_width
                aoi_label_y = mask_upper_right_viewport_coordinates[1] / self._ctx.viewport_height
                if self._img_size_fraction == 0:
                    # move offscreen (yes, it'd be better to hide it but we don't have an API method for that)
                    self._aoi_labels[image_idx].set_position(-10, -10, False, True)
                else:
                    self._aoi_labels[image_idx].set_position(aoi_label_x, aoi_label_y, False, True)

    def mouse_pos_handler(self, ctx: WindowCtx, x: float, y: float) -> bool:
        if self._mouse_down_image is not None and self._current_selection:
            selection = self._current_selection[self._mouse_down_image]
            if selection.finalized:
                return True

            # TODO[tws] add accessors for image width, height to PointViz
            image_width = selection._sensor._meta.w
            image_height = selection._sensor._meta.h
            pixel = selection._image.viewport_coordinates_to_image_pixel(ctx, x, y)

            # clamp pixel to a valid location in the image
            pixel = (
                max(0, min(pixel[0], image_height)),
                max(0, min(pixel[1], image_width))
            )
            # update top and bottom image selections regardless of which one we're drawing the AOI
            for selection in self._current_selection:
                # make sure p1 and p2 are not the same point on each axis
                first_same = selection.p1[0] == pixel[0]
                second_same = selection.p1[1] == pixel[1]
                if first_same:
                    pixel = (selection.p1[0] + 1, pixel[1])
                if second_same:
                    pixel = (pixel[0], selection.p1[1] + 1)
                selection.p2 = pixel

            # providing the window context here is a necessary evil to compute window coordinates
            # of the image
            self.update_aoi(ctx)
            return False

        if self._pick_start is not None:
            # update the mask
            x2 = int(ctx.mouse_x)
            y2 = int(ctx.mouse_y)
            x, y = self._pick_start

            # enqueue another pick request
            self._move_queue.put((self._queue_idx, x, y, x2, y2, False), block=False)
            self._queue_idx += 1

            return True
        return True

    def update_aoi_label(self, frames: FrameSet):
        self._frame_set = frames
        if not self._current_selection:
            [label.set_text("") for label in self._aoi_labels]  # type: ignore
            return

        for selection_idx, selection in enumerate(self._current_selection):
            sensor_idx = selection._sensor_index
            frame = frames[sensor_idx]
            if not frame:
                return

            # get filters for valid returns (range) that are within the AOI
            meta = selection._sensor._meta
            valid_aoi = core.destagger(
                meta, (selection._aoi_mask > 0).astype(dtype=np.uint8), inverse=True
            )

            valid = valid_aoi > 0

            # image_index = self._current_selection._image_index  # FIXME support both indices?
            key = self._image_mode_names[selection._image_index]

            # consider only valid values to avoid corrupting the statistical calculations
            # with invalid 0 values
            if key in [ChanField.RANGE, ChanField.RANGE2, ChanField.REFLECTIVITY,
                       ChanField.REFLECTIVITY2, ChanField.SIGNAL, ChanField.SIGNAL2,
                       ChanField.NEAR_IR]:
                valid_field = frame.field(key) > 0
                valid &= valid_field

            if not frame.has_field(key):
                continue

            field_data = frame.field(key)
            data = field_data[valid]
            label_str = f"{str(key).replace('_', '-')}\n"

            p1 = selection.p1
            p2 = selection.p2
            # subtract one from the max values
            # so that we show the inclusive range rather than the slice into the numpy array
            min_x = min(p1[0], p2[0])
            max_x = max(p1[0], p2[0]) - 1
            min_y = min(p1[1], p2[1])
            max_y = max(p1[1], p2[1]) - 1

            field_shape = field_data.shape
            channel_dimension = field_shape[2] if len(field_shape) == 3 else 1

            data = data.reshape(-1, channel_dimension)
            if data.size == 0:
                data = np.zeros((1, channel_dimension), dtype=np.float32)

            # Use float32 accumulators for fp16 RGB stats; summing/squaring AOI
            # pixels in float16 can overflow even when the source values are valid.
            stats_dtype = (np.float32 if np.issubdtype(data.dtype, np.floating) and
                           data.dtype.itemsize < np.dtype(np.float32).itemsize else None)
            comp_min = np.min(data, axis=0)
            comp_max = np.max(data, axis=0)
            comp_mean = np.mean(data, axis=0, dtype=stats_dtype)
            comp_std = np.std(data, axis=0, dtype=stats_dtype)

            is_vector = channel_dimension == 3

            if is_vector:
                def format_values(values: np.ndarray) -> str:
                    return "[" + ", ".join(f"{v:6.3f}" for v in values) + "]"

                stats_lines = [
                    ("min/max:", format_values(comp_min), format_values(comp_max)),
                    ("mean:", format_values(comp_mean)),
                    ("std:", format_values(comp_std)),
                ]
            else:
                def format_values(values: np.ndarray) -> str:
                    return f"{values[0]:8.1f}"

                stats_lines = [
                    ("min/max:", format_values(comp_min), format_values(comp_max)),
                    ("mean/std:", format_values(comp_mean), format_values(comp_std)),
                ]

            for prefix, *values in stats_lines:
                value_str = " ".join(values).rstrip()
                label_str += f"{prefix:<9}{value_str}\n"

            label_str += f"[{min_y}:{max_y} {min_x}:{max_x}]\n"
            self._aoi_labels[selection._image_index].set_text(label_str)

    def flip_images(self, flip: bool) -> None:
        self._flip_images = flip
        self.update_image_size(0)

    def toggle_flip_images(self) -> None:
        self.flip_images(not self._flip_images)

    def show_one_image(self, do: bool) -> None:
        self._show_one_image = do
        self.update_image_size(0)

    def update_image_size(self, amount: int) -> None:
        """Change the size of the 2D image and position image labels."""
        size_fraction_max = 20
        self._img_size_fraction = (self._img_size_fraction + amount +
                                   (size_fraction_max + 1)) % (
                                       size_fraction_max + 1)
        enabled_sensors = [sensor for sensor in self._sensors if sensor._enabled]
        image_h = self._img_size_fraction / size_fraction_max

        # compute the total width
        total_image_w = 0.0
        for sensor in enabled_sensors:
            image_w = image_h / sensor._img_aspect_ratio
            total_image_w += image_w

        # set image positions
        center_x = -total_image_w / 2
        last_image_right = 0.0
        for sensor in enabled_sensors:
            image_w = image_h / sensor._img_aspect_ratio
            image_left = last_image_right
            image_right = last_image_right + image_w

            last_image_right = image_right
            if self._flip_images:
                image_right, image_left = image_left, image_right
            for image_idx, image in enumerate(sensor._images):
                image_top = 1.0 - image_h * image_idx
                image_bottom = 1.0 - image_h * (image_idx + 1)
                if self._flip_images:
                    image_top, image_bottom = image_bottom, image_top
                if self._show_one_image and image_idx > 0:
                    image.set_position(1000, 0, 0, 0)
                else:
                    image.set_position(image_left + center_x, image_right + center_x, image_bottom, image_top)

                if image_idx == 0:
                    sensor._object_overlay.set_position(
                        image_left + center_x,
                        image_right + center_x,
                        image_bottom,
                        image_top
                    )
        self.update_aoi()

    def update_overlay_colors(self, palette):
        # TODO[tws] deduplicate logic with that found in update_objects
        self._flags_palette = palette
        for obj, cuboid in self._objects.items():
            color_attr = obj.id if self._object_color_mode == ObjectColorMode.OBJECT_ID else obj.class_id
            color = self._flags_palette[color_attr % len(palette)]
            selected_color = (*color[0:3], 0.9)
            if obj.id in self._object_labels:
                cuboid.set_rgba(selected_color)
            else:
                cuboid.set_rgba((*color, 0.3))

        # necessary to update the cuboids in the overlay,
        # which at present do not share a reference with the cuboids in the main viz
        self.update_object_cuboids()


# ``LidarScanVizModel`` was renamed to ``LidarFrameVizModel`` in the scan -> frame
# migration. Expose the old name (with a deprecation warning) so existing call
# sites keep working.
deprecated_alias("LidarScanVizModel", "LidarFrameVizModel", LidarFrameVizModel, globals(), "1.0")
