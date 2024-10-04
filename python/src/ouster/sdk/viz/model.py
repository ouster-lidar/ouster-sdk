"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Types for managing visualization state.
"""

import copy
from dataclasses import dataclass
import numpy as np
from typing import Any, Callable, Dict, List, Optional, Set

from .view_mode import (ImageMode, CloudMode, LidarScanVizMode,
                        SimpleMode, ReflMode, RGBMode,
                        is_norm_reflectivity_mode, CloudPaletteItem)
from ouster.sdk._bindings.viz import (Cloud, Image,
                   calref_palette, spezia_palette, spezia_cal_ref_palette, grey_palette,
                   grey_cal_ref_palette, viridis_palette, viridis_cal_ref_palette,
                   magma_palette, magma_cal_ref_palette)
import ouster.sdk.client as client
from ouster.sdk.client import (ChanField, ShotLimitingStatus, ThermalShutdownStatus)
from ouster.sdk.util import img_aspect_ratio


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
               info: Optional[client.SensorInfo] = None) -> LidarScanVizMode:
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


class Palettes:
    """Represents the color palettes used within an instance of LidarScanViz.
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
        ]

        self._refl_cloud_palettes: List[CloudPaletteItem]
        self._refl_cloud_palettes = [
            CloudPaletteItem("Cal. Ref", calref_palette),
            CloudPaletteItem("Cal. Ref. Ouster Colors",
                             spezia_cal_ref_palette),
            CloudPaletteItem("Cal. Ref. Greyscale", grey_cal_ref_palette),
            CloudPaletteItem("Cal. Ref. Viridis", viridis_cal_ref_palette),
            CloudPaletteItem("Cal. Ref. Magma", magma_cal_ref_palette),
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


class SensorModel:
    """
    A model object representing viz state for a single sensor.
    """

    def __init__(self, meta, *, _img_aspect_ratio: float = 0):
        self._enabled = True
        self._meta = meta
        self._xyzlut = client.XYZLut(meta, use_extrinsics=True)

        self._num_clouds = 2
        self._clouds: List[Cloud] = []
        self._cloud_masks: List[np.ndarray] = []
        for i in range(self._num_clouds):
            self._clouds.append(Cloud(meta))
            mask = np.zeros((meta.h, meta.w, 4), dtype=np.float32)
            mask[:, :, 1] = 1.0
            self._cloud_masks.append(mask)
        self._cloud_modes: Dict[str, CloudMode] = {}

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

        # TODO[tws] decide whether it's necessary to provide extra modes via the constructor
        # self._modes.extend(_ext_modes or [])

        self._populate_image_cloud_modes()
        self._image_calref_palette = copy.deepcopy(calref_palette)
        self._image_calref_palette[0] = [0.1, 0.1, 0.1]

    def _populate_image_cloud_modes(self) -> None:
        # TODO[tws] de-duplicate with logic in _amend_view_modes
        self._image_modes = {name:
            ImgModeItem(mode, name, num) for mode in self._modes
            if isinstance(mode, ImageMode)
            for num, name in enumerate(mode.names)
        }

        self._cloud_modes = {m.name:
            m for m in self._modes if isinstance(m, CloudMode)}

    def _create_view_mode_for_field(self, field_name, scan) -> Optional[LidarScanVizMode]:
        """Create the appropriate view mode depending on a field's name and dimensions."""
        mode: Optional[LidarScanVizMode] = None
        if field_name in [ChanField.FLAGS2, ChanField.RANGE2, ChanField.REFLECTIVITY2, ChanField.SIGNAL2]:
            return None
        if field_name in scan.fields:
            field = scan.field(field_name)
            f_shape = field.shape
            if len(f_shape) == 3 and f_shape[2] == 3:   # 3D field
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

    def update_cloud_palettes(self, cloud_mode_name, palettes: Palettes) -> None:
        """Sets each cloud palette given the cloud mode name.
        Has no effect if the cloud mode is not available for this sensor."""
        cloud_mode = self._cloud_modes.get(cloud_mode_name)
        if cloud_mode:
            palette = palettes.get_palette(cloud_mode)
            for cloud in self._clouds:
                cloud.set_palette(palette.palette)

    def update_cloud(self, cloud, cloud_mode, range_field: str, return_num: int, scan: client.LidarScan) -> None:
        """Updates the given Cloud with the given CloudMode."""
        if range_field in scan.fields:
            range_data = scan.field(range_field)
        else:
            range_data = np.zeros((scan.h, scan.w), dtype=np.uint32)

        if not cloud_mode:
            return

        cloud.set_column_poses(scan.pose)
        cloud.set_range(range_data)

        if cloud_mode.enabled(scan, return_num):
            cloud_mode.set_cloud_color(cloud, scan, return_num=return_num)
        else:
            cloud_mode.set_cloud_color(cloud, scan, return_num=0)

    def update_clouds(self, cloud_mode_name: str, scan: Optional[client.LidarScan]) -> None:
        """Update range and mode for each cloud given a mode name and a scan."""

        if not scan:
            return

        cloud_mode = self._cloud_modes.get(cloud_mode_name, None)
        for return_num, range_field in ((0, ChanField.RANGE), (1, ChanField.RANGE2)):
            if return_num < len(self._clouds):
                self.update_cloud(self._clouds[return_num], cloud_mode, range_field, return_num, scan)

    def update_image(self, image, image_mode_item: ImgModeItem, scan: Optional[client.LidarScan]) -> None:
        """Update the view mode of the given image."""
        # TODO[tws] optimize, move to image model(?)

        if not scan:
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

    def update_images(self, image_mode_names: List[str], scan: Optional[client.LidarScan]) -> None:
        """Update image values and mode given mode names and a scan."""
        assert len(image_mode_names) == len(self._images)
        for i in range(len(self._images)):
            img_mode_item = self._image_modes.get(image_mode_names[i])
            if img_mode_item:
                self.update_image(self._images[i], img_mode_item, scan)
            else:
                # TODO[tws]: deduplicate, e.g. by making this a method in an image model class
                self._images[i].set_image(np.zeros((self._meta.h, self._meta.w), dtype=np.float32))


class LidarScanVizModel:
    def __init__(self, metas: List[client.SensorInfo], *, _img_aspect_ratio: float):
        self._cloud_pt_size: float = 2.0
        self._metas = metas
        assert len(metas) > 0, "ERROR: Expecting at least one sensor"
        self._sensors: List[SensorModel] = [
            SensorModel(meta, _img_aspect_ratio=_img_aspect_ratio) for meta in self._metas
        ]
        self._max_clouds = max(sensor._num_clouds for sensor in self._sensors)
        self._max_images = max(sensor._num_images for sensor in self._sensors)
        self._cloud_mode_ind: int = 0
        self._cloud_mode_name: str = ''
        self._cloud_enabled = [True, True]
        self._image_mode_ind = [0] * self._max_images
        self._image_mode_names = [''] * self._max_images
        self._set_cloud_pt_size(self._cloud_pt_size)
        self._known_fields: Set[str] = set()              # set of known fields
        _ext_palettes: List[CloudPaletteItem] = []   # TODO[tws] implement
        self._palettes: Palettes = Palettes(_ext_palettes)
        self._cloud_palette_name = ''

    def _set_cloud_pt_size(self, point_size):
        for sensor in self._sensors:
            for cloud in sensor._clouds:
                cloud.set_point_size(point_size)

    # TODO[tws] likely remove
    @property
    def metadata(self) -> List[client.SensorInfo]:
        """Metadatas for the displayed sensors."""
        return self._metas

    def _amend_view_modes_all(self, scans: List[Optional[client.LidarScan]]) -> None:
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
            sensor.update_cloud_palettes(self._cloud_mode_name, self._palettes)

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

    def cycle_cloud_mode(self, direction: int):
        """Updates the currently selected cloud mode from the list of all available cloud modes."""
        all_cloud_mode_names = self.sorted_cloud_mode_names()
        self._cloud_mode_ind = (self._cloud_mode_ind + direction) % len(all_cloud_mode_names)
        self._cloud_mode_name = all_cloud_mode_names[self._cloud_mode_ind]
        self.update_cloud_palettes()

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

    def update(self, scans: List[Optional[client.LidarScan]]) -> None:
        """Update the LidarScanViz state with the provided scans."""
        assert len(scans) == len(self._sensors)

        self._amend_view_modes_all(scans)

        for scan, sensor in zip(scans, self._sensors):
            if not sensor._enabled:
                continue
            sensor.update_clouds(self._cloud_mode_name, scan)
            sensor.update_images(self._image_mode_names, scan)

            if scan:
                # print warnings
                first_ts_s = client.first_valid_column_ts(scan) / 1e9
                if scan.shot_limiting() != ShotLimitingStatus.SHOT_LIMITING_NORMAL:
                    print(f"WARNING: Shot limiting status: {scan.shot_limiting()} "
                        f"(sensor ts: {first_ts_s:.3f})")
                if scan.thermal_shutdown() != ThermalShutdownStatus.THERMAL_SHUTDOWN_NORMAL:
                    print(f"WARNING: Thermal shutdown status: {scan.thermal_shutdown()} "
                        f"(sensor ts: {first_ts_s:.3f})")
