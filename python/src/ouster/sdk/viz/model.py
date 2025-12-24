"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Types for managing visualization state.
"""

import copy
from dataclasses import dataclass
import numpy as np
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

from .view_mode import (ImageMode, CloudMode, LidarScanVizMode,
                        SimpleMode, ReflMode, RGBMode, NormalsMode, RingMode,
                        is_norm_reflectivity_mode, CloudPaletteItem)
from ouster.sdk._bindings.viz import (Cloud, Image, Label, PointViz, Mesh,
                   calref_palette, spezia_palette, spezia_cal_ref_palette, grey_palette,
                   grey_cal_ref_palette, viridis_palette, viridis_cal_ref_palette,
                   magma_palette, magma_cal_ref_palette, MouseButton, MouseButtonEvent, EventModifierKeys, WindowCtx)
import ouster.sdk.core as core
from ouster.sdk.core import (ChanField, ShotLimitingStatus, ThermalShutdownStatus, LidarScanSet)
from ouster.sdk.util import img_aspect_ratio
from .widgets import ToggleImage, ToggleLabel, ToggleMesh
from ouster.sdk.zone_monitor import ZONE_OCCUPANCY_FIELDNAME, ZONE_STATES_FIELDNAME, CoordinateFrame
from ouster.sdk.viz.mesh import clouds_from_zrb, voxel_style_mesh_from_zrb
from ouster.sdk._bindings.viz import precompute_voxel_vertices
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


def _flatten(xss: List[List[Any]]) -> List[Any]:
    """Flattens a list of lists."""
    return [x for xs in xss for x in xs]


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
    func: Callable[[], LidarScanVizMode]

    def create(self,
               info: Optional[core.SensorInfo] = None) -> LidarScanVizMode:
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


class Palettes:
    """Represents the color palettes used within an instance of LidarScanViz.
    Also keeps track of the palette currently in use."""

    def __init__(self, _ext_palettes: List[CloudPaletteItem]):
        """Initialize a Palettes object, which populates two lists
        of palettes - one for normal view modes and one for ReflMode."""

        # Generate a rainbow palette
        rainbow_palette = np.zeros((256, 3))
        for i in range(0, 256):
            res = _hsv_to_rgb(int(i * 230 / 255), 255, 255)
            rainbow_palette[i, 0] = res[0] / 255
            rainbow_palette[i, 1] = res[1] / 255
            rainbow_palette[i, 2] = res[2] / 255

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


class SensorModel:
    """
    A model object representing viz state for a single sensor.
    """

    def __init__(self, viz: PointViz, meta, *, _img_aspect_ratio: float = 0, palettes: Palettes = Palettes([])):
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
            self._clouds.append(Cloud(meta))
            mask = np.zeros((meta.h, meta.w, 4), dtype=np.float32)
            mask[:, :, 1] = 1.0
            self._cloud_masks.append(mask)
        self._cloud_modes: Dict[str, CloudMode] = {}

        self._palette_dirty = [True] * len(self._clouds)

        self._num_images = 2
        self._images: List[Image] = []
        self._image_modes: Dict[str, ImgModeItem] = {}
        self._image_labels = [None] * self._num_images
        for i in range(self._num_images):
            self._images.append(Image())

        if _img_aspect_ratio:
            self._img_aspect_ratio = _img_aspect_ratio
        else:
            self._img_aspect_ratio = img_aspect_ratio(meta)

        self._modes: List[LidarScanVizMode] = []

        # Add extra viz mode, usually inserted through plugins
        self._modes.extend([vm.create(meta) for vm in _viz_extra_modes])
        self._modes.append(RingMode(meta))

        # TODO[tws] decide whether it's necessary to provide extra modes via the constructor
        # self._modes.extend(_ext_modes or [])

        self._black_palette = np.zeros((256, 3), dtype=np.float32)

        self._populate_image_cloud_modes()
        self._image_calref_palette = copy.deepcopy(calref_palette)
        self._image_calref_palette[0] = [0.1, 0.1, 0.1]

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
                    zone_transform = meta.extrinsic @ np.linalg.inv(sensor_to_body_transform)
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
    def _get_field_class_for_field_name(field_name, scan) -> Optional[core.FieldType]:
        for field_type in scan.field_types:
            if field_type.name == field_name:
                return field_type.field_class
        return None

    def _create_view_mode_for_field(self, field_name, scan) -> Optional[LidarScanVizMode]:
        """Create the appropriate view mode depending on a field's name and dimensions."""
        mode: Optional[LidarScanVizMode] = None
        if field_name in [ChanField.FLAGS2, ChanField.RANGE2, ChanField.REFLECTIVITY2, ChanField.SIGNAL2]:
            return None

        if field_name in scan.fields:
            if SensorModel._get_field_class_for_field_name(field_name, scan) != core.FieldClass.PIXEL_FIELD:
                return None

            field = scan.field(field_name)
            f_shape = field.shape
            if len(f_shape) == 3 and f_shape[2] == 3:   # 3D field
                upper_name = field_name.upper()
                if upper_name.startswith("NORMALS"):
                    mode = NormalsMode(field_name, info=self._meta)
                else:
                    mode = RGBMode(field_name, info=self._meta)
            elif len(f_shape) == 2:
                if field_name == ChanField.REFLECTIVITY:
                    mode = ReflMode(info=self._meta)
                elif field_name == ChanField.NEAR_IR:
                    mode = SimpleMode(field_name, info=self._meta, use_ae=True, use_buc=True)
                else:
                    mode = SimpleMode(field_name, info=self._meta)
        return mode

    # TODO[tws] rename
    def _amend_view_modes(self, scan):
        """
        Update the available fields for this model given a scan.
        """
        if not self._has_imu_data:
            # TODO[tws] set based on imu visualization config maybe?
            if scan.has_field('IMU_GYRO') or scan.has_field('IMU_ACC'):
                self._has_imu_data = True

        for field_name in scan.fields:
            # skip if view mode already exists
            if field_name in self._image_modes or \
               field_name == ChanField.FLAGS or \
               field_name == ChanField.FLAGS2:
                continue
            mode = self._create_view_mode_for_field(field_name, scan)
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

    def update_cloud(self, cloud, cloud_mode, range_field: str, return_num: int,
                     scan: Optional[core.LidarScan]) -> None:
        """Updates the given Cloud with the given CloudMode."""
        if scan is None:
            # make the scan invisible if it isnt present
            cloud.set_range(np.zeros((cloud.cols, cloud.size // cloud.cols), dtype=np.uint32))
            return

        if range_field in scan.fields:
            range_data = scan.field(range_field)
        else:
            range_data = np.zeros((scan.h, scan.w), dtype=np.uint32)

        cloud.set_column_poses(scan.pose)
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

        if cloud_mode.enabled(scan, return_num):
            cloud_mode.set_cloud_color(cloud, scan, return_num=return_num)
        else:
            cloud_mode.set_cloud_color(cloud, scan, return_num=0)

    def update_clouds(self, cloud_mode_name: str, scan: Optional[core.LidarScan]) -> None:
        """Update range and mode for each cloud given a mode name and a scan."""
        cloud_mode = self._cloud_modes.get(cloud_mode_name, None)
        for return_num, range_field in ((0, ChanField.RANGE), (1, ChanField.RANGE2)):
            if return_num < len(self._clouds):
                self.update_cloud(self._clouds[return_num], cloud_mode, range_field, return_num, scan)

    def update_image(self, image, image_mode_item: ImgModeItem, scan: Optional[core.LidarScan]) -> None:
        """Update the view mode of the given image."""
        # TODO[tws] optimize, move to image model(?)

        if not scan:
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

        if mode is not None and mode.enabled(scan, image_mode_item.return_num):
            mode.set_image(image, scan, image_mode_item.return_num)
        else:
            # TODO[tws]: deduplicate, e.g. by making this a method in an image model class
            image.set_image(np.zeros((self._meta.h, self._meta.w), dtype=np.float32))

    def update_images(self, image_mode_names: List[str], scan: Optional[core.LidarScan]) -> None:
        """Update image values and mode given mode names and a scan."""
        assert len(image_mode_names) == len(self._images)
        for i in range(len(self._images)):
            image_mode_item = self._image_modes.get(image_mode_names[i])
            if image_mode_item:
                self.update_image(self._images[i], image_mode_item, scan)
            else:
                # TODO[tws]: deduplicate, e.g. by making this a method in an image model class
                self._images[i].set_image(np.zeros((self._meta.h, self._meta.w), dtype=np.float32))

    def update_zones(self, scan: Optional[core.LidarScan] = None) -> None:
        if self._meta.zone_set is None:
            return

        # TODO[tws] use >= max zone id constant here
        live_zone_ids: Set[int] = set()
        if self._zone_states is not None:
            live_zone_ids = {zone_state.id for zone_state in self._zone_states if zone_state.id < 255}

        zrb_pose: Optional[np.ndarray] = None
        stl_pose: Optional[np.ndarray] = None
        if scan is not None:
            pose = scan.pose[scan.get_last_valid_column()]
            zrb_pose = pose
            zrb_mesh_pose = (pose @ self._meta.extrinsic)
            stl_pose = (pose @ self._meta.extrinsic) @ np.linalg.inv(self._meta.zone_set.sensor_to_body_transform)

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
                        stl_mesh._item.set_transform(zrb_pose)

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
                zrb_mesh._item.set_transform(self._meta.extrinsic)
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

    def update_zone_occupancy_cloud_and_image_masks(self, scan: Optional[core.LidarScan], palettes: Palettes):
        if not scan:
            # TODO[tws] deactivate zones / clear masks
            return

        destaggered_mask = mask = np.zeros((self._meta.h, self._meta.w, 4), dtype=np.float32)
        if scan.has_field(ZONE_OCCUPANCY_FIELDNAME):
            occupancy = scan.field(ZONE_OCCUPANCY_FIELDNAME)
            # TODO[tws] consider flags mode.
            # TODO[tws] color mask depending on zone color?
            if scan.has_field(ZONE_STATES_FIELDNAME):
                zone_states = scan.field(ZONE_STATES_FIELDNAME)
                for z_idx in range(16):  # TODO[tws] max live zones constant
                    zone_state = zone_states[z_idx]
                    zone_id = zone_state.id
                    # TODO[tws] use >= max zone id constant here
                    if zone_id == 255:
                        continue
                    bit = 0x1 << z_idx
                    mask[(occupancy & bit) > 0] = triggered_live_zone_color(self._zone_palette, zone_id)
            else:
                mask[occupancy > 0] = (1, 0, 1, 1)
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


class LidarScanVizModel:
    def __init__(self, viz: PointViz, metas: List[core.SensorInfo], *, _img_aspect_ratio: float):
        self._viz = viz
        self._cloud_pt_size: float = 2.0
        self._metas = metas
        assert len(metas) > 0, "ERROR: Expecting at least one sensor"
        _ext_palettes: List[CloudPaletteItem] = []   # TODO[tws] implement
        self._palettes: Palettes = Palettes(_ext_palettes)
        self._sensors: List[SensorModel] = [
            SensorModel(viz, meta, _img_aspect_ratio=_img_aspect_ratio, palettes=self._palettes) for meta in self._metas
        ]
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

        # AOI-related
        self._mouse_down_image: Optional[int] = None
        self._current_selection: List[Selection2d] = []
        self._aoi_labels = [Label("", 0, 0) for _ in range(self._max_images)]
        self._preexisting_selection: bool = False

        # regrettable, since images aren't always aware of the context they're rendered in
        self._ctx: Optional[WindowCtx] = None
        self._img_size_fraction = 4

        self._flip_images = False
        self._show_one_image = False

    def _set_cloud_pt_size(self, point_size):
        for sensor in self._sensors:
            for cloud in sensor._clouds:
                cloud.set_point_size(point_size)

    # TODO[tws] likely remove
    @property
    def metadata(self) -> List[core.SensorInfo]:
        """Metadatas for the displayed sensors."""
        return self._metas

    def _amend_view_modes_all(self, scans: LidarScanSet) -> None:
        """Add new image and cloud view modes to each sensor for each any new fields in the received scans."""

        assert len(scans) == len(self._sensors)
        for scan, sensor in zip(scans, self._sensors):
            if not scan:
                continue
            # TODO[tws] optimize; _known_fields is only used to check for
            # whether preferred fields are available for default view modes
            self._known_fields.update(scan.fields)
            sensor._amend_view_modes(scan)

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
        """Returns the image mode names, limited to those for fields we've seen in at least one LidarScan."""
        return sorted(
            list(
                set(
                    _flatten([sensor._image_modes.keys() for sensor in self._sensors])
                ) & self._known_fields  # limit to fields seen in LidarScans
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

        try:
            preferred_fields = [ChanField.REFLECTIVITY, ChanField.REFLECTIVITY2]
            if ChanField.REFLECTIVITY2 not in self._known_fields:
                preferred_fields[1] = ChanField.NEAR_IR
            self._cloud_mode_ind = sorted_cloud_mode_names.index(preferred_fields[0])
            self._image_mode_ind[0] = sorted_image_mode_names.index(preferred_fields[0])
            self._image_mode_ind[1] = sorted_image_mode_names.index(preferred_fields[1])
        except ValueError:
            # handle a situation where no reflectivity or near ir channels are present
            # which may happen with customized datasets
            self._cloud_mode_ind = 0
            self._image_mode_ind = [0, 0]

        self._cloud_mode_name = sorted_cloud_mode_names[self._cloud_mode_ind]
        self._image_mode_names[0] = sorted_image_mode_names[self._image_mode_ind[0]]
        self._image_mode_names[1] = sorted_image_mode_names[self._image_mode_ind[1]]
        self.update_cloud_palettes()

    def update(self, scans: LidarScanSet) -> None:
        """Update the LidarScanViz state with the provided scans."""
        assert len(scans) == len(self._sensors)

        self._amend_view_modes_all(scans)

        for scan, sensor in zip(scans, self._sensors):
            sensor.update_clouds(self._cloud_mode_name, scan)
            sensor.update_images(self._image_mode_names, scan)

            if scan:
                if scan.has_field(ZONE_STATES_FIELDNAME):
                    # a zero timestamp indicates we didnt get a packet
                    if scan.has_field(ChanField.ZONE_PACKET_TIMESTAMP) and \
                       scan.field(ChanField.ZONE_PACKET_TIMESTAMP)[0] == 0:
                        sensor._zone_states = None
                    else:
                        sensor._zone_states = scan.field(ZONE_STATES_FIELDNAME)
                sensor.update_zones(scan)

                # check if we should enable dual return mode
                if len(self._cloud_enabled) == 1:
                    if scan.has_field('RANGE2'):
                        self._cloud_enabled.append(True)
                # print warnings
                first_ts_s = scan.get_first_valid_column_timestamp() / 1e9
                if scan.shot_limiting() != ShotLimitingStatus.NORMAL:
                    print(f"WARNING: Shot limiting status: {scan.shot_limiting()} "
                        f"(sensor ts: {first_ts_s:.3f})")
                if scan.thermal_shutdown() != ThermalShutdownStatus.NORMAL:
                    print(f"WARNING: Thermal shutdown status: {scan.thermal_shutdown()} "
                        f"(sensor ts: {first_ts_s:.3f})")

    def clear_masks(self):
        for sensor_index, sensor in enumerate(self._sensors):
            for image_idx, image in enumerate(sensor._images):
                meta = sensor._meta
                mask = np.zeros((meta.h, meta.w, 4), dtype=np.float32)
                image.set_mask(mask)
                sensor._clouds[image_idx].set_mask(mask)

    def clear_aoi(self):
        self._current_selection = []
        [aoi_label.set_text("") for aoi_label in self._aoi_labels]
        self.clear_masks()

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

    def mouse_button_handler(self, ctx: WindowCtx,
        button: MouseButton, event: MouseButtonEvent, mods: EventModifierKeys) -> bool:

        ret = True
        if button == MouseButton.MOUSE_BUTTON_RIGHT and event == MouseButtonEvent.MOUSE_BUTTON_PRESSED \
            and self._img_size_fraction > 0:
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
        return ret

    def update_aoi(self, ctx: Optional[WindowCtx] = None):
        if not self._current_selection:
            return
        if ctx:
            self._ctx = ctx

        # clear the masks - important because the image mode may have changed
        self.clear_masks()

        for selection_idx, selection in enumerate(self._current_selection):
            image_idx = selection._image_index
            sensor = selection._sensor
            image = sensor._images[image_idx]
            meta = sensor._meta
            p1 = selection.p1
            p2 = selection.p2

            # compute the image mask given the selection area
            mask = np.zeros((meta.h, meta.w, 4), dtype=np.float32)
            min_x = min(p1[0], p2[0])
            max_x = max(p1[0], p2[0])
            min_y = min(p1[1], p2[1])
            max_y = max(p1[1], p2[1])
            mask[min_x:max_x, min_y:max_y, :] = (1, 0, 0, 0.6)
            image.set_mask(mask)

            # compute the cloud mask
            cloud_mask = np.zeros((meta.h, meta.w, 4), dtype=np.float32)
            cloud_mask[min_x:max_x, min_y:max_y, :] = (1, 0, 0, 0.8)
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
        return True

    def update_aoi_label(self, scans: LidarScanSet):
        if not self._current_selection:
            [label.set_text("") for label in self._aoi_labels]  # type: ignore
            return

        for selection_idx, selection in enumerate(self._current_selection):
            sensor_idx = selection._sensor_index
            scan = scans[sensor_idx]
            if not scan:
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
                valid_field = scan.field(key) > 0
                valid &= valid_field

            if not scan.has_field(key):
                continue

            field_data = scan.field(key)
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

            comp_min = np.min(data, axis=0)
            comp_max = np.max(data, axis=0)
            comp_mean = np.mean(data, axis=0)
            comp_std = np.std(data, axis=0)

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
        self.update_aoi()
