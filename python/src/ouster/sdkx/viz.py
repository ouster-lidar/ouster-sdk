#  type: ignore
"""Internal visualizer tools."""

from enum import Enum
from typing import Optional, List

import numpy as np

from ouster import client
from ouster.client import _utils, ChanField
from ouster.sdk.viz import (PointViz, LidarScanViz, WindowCtx, Image, Cloud,
                            push_point_viz_handler, ImageCloudMode,
                            CloudPaletteItem, magma_palette)
from ouster.sdkx.util import img_aspect_ratio


class RGBMode(ImageCloudMode):
    """View mode to use RGB channels"""

    def __init__(self,
                 info: client.SensorInfo,
                 rgb_fields: Optional[List[client.ChanField]] = [],
                 *,
                 prefix: Optional[str] = "",
                 suffix: Optional[str] = "",
                 use_buc: bool = False) -> None:
        self._info = info
        self._rgb_fields = []
        self._bucs = []

        if rgb_fields:
            if len(rgb_fields) != 3:
                raise AssertionError("rgb_fields should have 3 ChanField "
                                     "elements")
            self._rgb_fields = rgb_fields
            if use_buc:
                self._bucs = [
                    _utils.BeamUniformityCorrector() for _ in self._rgb_fields
                ]

        self._prefix = f"{prefix}: " if prefix else ""
        self._suffix = f" ({suffix})" if suffix else ""
        self._wrap_name = lambda n: f"{self._prefix}{n}{self._suffix}"

    @property
    def name(self) -> str:
        return self._wrap_name("RGB")

    @property
    def names(self) -> str:
        return [self.name]

    def _prepare_data(self,
                      ls: client.LidarScan,
                      return_num: int = 0) -> Optional[np.ndarray]:
        if not self.enabled(ls, return_num):
            return None

        r = ls.field(self._rgb_fields[0])
        g = ls.field(self._rgb_fields[1])
        b = ls.field(self._rgb_fields[2])

        normalizer = 255

        # for types other than uint8 for RED, GREEN, BLUE channels
        # we try to check are there really value bigger than 255
        if r.dtype != np.uint8 or g.dtype != np.uint8 or b.dtype != np.uint8:
            max_rgb = np.max((np.max(r), np.max(g), np.max(b)))
            if max_rgb > 255:
                normalizer = 65535

        r = (r / normalizer).clip(0, 1.0).astype(float)
        g = (g / normalizer).clip(0, 1.0).astype(float)
        b = (b / normalizer).clip(0, 1.0).astype(float)

        if self._bucs:
            self._bucs[0](r)
            self._bucs[1](g)
            self._bucs[2](b)

        rgb = np.dstack((r, g, b))

        return rgb

    def set_image(self,
                  img: Image,
                  ls: client.LidarScan,
                  return_num: int = 0) -> None:
        key_data = self._prepare_data(ls, return_num)
        if key_data is not None:
            img.set_image(client.destagger(self._info, key_data))

    def set_cloud_color(self,
                        cloud: Cloud,
                        ls: client.LidarScan,
                        return_num: int = 0) -> None:
        key_data = self._prepare_data(ls, return_num)
        if key_data is not None:
            cloud.set_key(key_data)

    def enabled(self, ls: client.LidarScan, return_num: int = 0):
        if len(self._rgb_fields) == 3 and return_num == 0:
            return all(f in ls.fields for f in self._rgb_fields)
        return False


class ExtendedScanViz(LidarScanViz):
    """Add bloom highlighting and hiding to LidarScanViz.

    This class provides additional functionality used in fw development. It's
    currently used by ouster-cli, which is internal-only.

    Adds a key binding to "c" to toggle the "flags mode". This will cycle
    between highlighting the second return, highlighting pixels where the bloom
    bit is set, and hiding pixels where the bloom bit is set. The first option
    won't do anything in single returns mode, and the latter two won't do
    anything unless the sensor is configured to disable bloom reduction.
    """

    class FlagsMode(Enum):
        NONE = 0
        HIGHLIGHT_SECOND = 1
        HIGHLIGHT_BLOOM = 2
        HIDE_BLOOM = 3

    def __init__(self,
                 meta: client.SensorInfo,
                 viz: Optional[PointViz] = None,
                 _img_aspect_ratio: float = 0) -> None:
        # add additional image/cloud modes
        ext_modes = [
            RGBMode(meta,
                    rgb_fields=[
                        ChanField.CUSTOM0, ChanField.CUSTOM1, ChanField.CUSTOM2
                    ]),
            RGBMode(meta,
                    rgb_fields=[
                        ChanField.CUSTOM0, ChanField.CUSTOM1, ChanField.CUSTOM2
                    ],
                    suffix="w BUC",
                    use_buc=True)
        ]
        # add additional color palettes for cloud points
        magma_bright_palette = np.ones(magma_palette.shape)
        magma_bright_palette[:-64] = magma_palette[64:]
        magma_bright_palette[-64:] = magma_palette[-1]
        ext_palettes = [
            CloudPaletteItem("Magma Bright", magma_bright_palette)
        ]
        aspect_ratio = _img_aspect_ratio or img_aspect_ratio(meta)
        super().__init__(meta,
                         viz or PointViz("Extended Ouster Viz"),
                         _img_aspect_ratio=aspect_ratio,
                         _ext_modes=ext_modes,
                         _ext_palettes=ext_palettes)

        # initialize masks to just green with zero opacity
        mask = np.zeros(
            (meta.format.pixels_per_column, meta.format.columns_per_frame, 4),
            dtype=np.float32)
        mask[:, :, 1] = 1.0

        self._cloud_masks = (mask, mask.copy())
        self._flags_mode = ExtendedScanViz.FlagsMode.NONE

        def key_handler(self: ExtendedScanViz, ctx: WindowCtx, key: int,
                        mods: int) -> bool:
            if key == ord('C') and mods == 0:
                self.update_flags_mode(None)
                self.draw()
            return True

        push_point_viz_handler(self._viz, self, key_handler)

    def _draw(self) -> None:
        # call original draw
        super()._draw()

        # set a cloud mask based where first flags bit is set
        if self._flags_mode == ExtendedScanViz.FlagsMode.HIGHLIGHT_BLOOM:
            for i, flag_field in ((0, ChanField.FLAGS), (1, ChanField.FLAGS2)):
                if flag_field in self._scan.fields:
                    mask_opacity = (self._scan.field(flag_field) & 0x1) * 1.0
                    self._cloud_masks[i][:, :, 3] = mask_opacity
                    self._clouds[i].set_mask(self._cloud_masks[i])

        # set range to zero where first flags bit is set
        elif self._flags_mode == ExtendedScanViz.FlagsMode.HIDE_BLOOM:
            for i, flag_field, range_field in ((0, ChanField.FLAGS,
                                                ChanField.RANGE),
                                               (1, ChanField.FLAGS2,
                                                ChanField.RANGE2)):
                if flag_field in self._scan.fields and range_field in self._scan.fields:
                    # modifying the scan in-place would break cycling modes while paused
                    range = self._scan.field(range_field).copy()
                    range[self._scan.field(flag_field) & 0x1 == 0x1] = 0
                    self._clouds[i].set_range(range)

    def update_flags_mode(self,
                          mode: 'Optional[ExtendedScanViz.FlagsMode]') -> None:
        with self._lock:
            # cycle between flag mode enum values
            if mode is None:
                self._flags_mode = ExtendedScanViz.FlagsMode(
                    (self._flags_mode.value + 1) %
                    len(ExtendedScanViz.FlagsMode.__members__))
            else:
                self._flags_mode = mode

            # set mask on all points in the second cloud, clear other mask
            if self._flags_mode == ExtendedScanViz.FlagsMode.HIGHLIGHT_SECOND:
                self._cloud_masks[0][:, :, 3] = 0.0
                self._cloud_masks[1][:, :, 3] = 1.0
                self._clouds[0].set_mask(self._cloud_masks[0])
                self._clouds[1].set_mask(self._cloud_masks[1])
            # clear masks on both clouds, expected to be set dynamically in _draw()
            else:
                self._cloud_masks[0][:, :, 3] = 0.0
                self._cloud_masks[1][:, :, 3] = 0.0
                self._clouds[0].set_mask(self._cloud_masks[0])
                self._clouds[1].set_mask(self._cloud_masks[1])

            # not bothering with OSD
            print("Flags mode:", self._flags_mode.name)
