from typing import (List, Optional, Union, Protocol, runtime_checkable)

from dataclasses import dataclass
import numpy as np
from ouster.sdk import core
from ouster.sdk.core import _utils, Version

from ouster.sdk._bindings.viz import Cloud, Image


@runtime_checkable
class FieldViewMode(Protocol):
    """LidarScan field processor

    View modes define the process of getting the key data for
    the scan and return number as well as checks the possibility
    of showing data in that mode, see `enabled()`.
    """

    _info: Optional[core.SensorInfo]

    @property
    def name(self) -> str:
        """Name of the view mode"""
        ...

    @property
    def names(self) -> List[str]:
        """Name of the view mode per return number"""
        ...

    def _prepare_data(self,
                      ls: core.LidarScan,
                      return_num: int = 0) -> Optional[np.ndarray]:
        """Prepares data for visualization given the scan and return number"""
        ...

    def enabled(self, ls: core.LidarScan, return_num: int = 0) -> bool:
        """Checks the view mode availability for a scan and return number"""
        ...


@runtime_checkable
class ImageMode(FieldViewMode, Protocol):
    """Applies the view mode key to the viz.Image"""

    def set_image(self,
                  img: Image,
                  ls: core.LidarScan,
                  return_num: int = 0) -> None:
        """Prepares the key data and sets the image key to it."""
        ...


@runtime_checkable
class CloudMode(FieldViewMode, Protocol):
    """Applies the view mode key to the viz.Cloud"""

    def set_cloud_color(self,
                        cloud: Cloud,
                        ls: core.LidarScan,
                        *,
                        return_num: int = 0) -> None:
        """Prepares the key data and sets the cloud key to it."""
        ...


class ImageCloudMode(ImageMode, CloudMode, Protocol):
    """Applies the view mode to viz.Cloud and viz.Image"""
    pass


def _second_chan_field(field: str) -> Optional[str]:
    """Get the second return field name."""
    # yapf: disable
    second_fields = dict({
        core.ChanField.RANGE: core.ChanField.RANGE2,
        core.ChanField.SIGNAL: core.ChanField.SIGNAL2,
        core.ChanField.REFLECTIVITY: core.ChanField.REFLECTIVITY2,
        core.ChanField.FLAGS: core.ChanField.FLAGS2
    })
    # yapf: enable
    return second_fields.get(field, None)


class RingMode(CloudMode):
    """View mode to show laser ring."""

    def __init__(self, info: core.SensorInfo) -> None:
        """
        Args:
            info: sensor metadata
        """
        self._info = info
        key_data = np.empty((info.h, info.w), dtype=np.float32)
        for i in range(0, info.h):
            key_data[i, :] = i / info.h
        self._key_data = key_data

    @property
    def name(self) -> str:
        return "RING"

    @property
    def names(self) -> List[str]:
        return ["RING"]

    def _prepare_data(self,
                      ls: core.LidarScan,
                      return_num: int = 0) -> Optional[np.ndarray]:
        return self._key_data

    def set_cloud_color(self,
                        cloud: Cloud,
                        ls: core.LidarScan,
                        return_num: int = 0) -> None:
        cloud.set_key(self._key_data)

    def enabled(self, ls: core.LidarScan, return_num: int = 0):
        return True


class SimpleMode(ImageCloudMode):
    """Basic view mode with AutoExposure and BeamUniformityCorrector

    Handles single and dual returns scans.

    When AutoExposure is enabled its state updates only for return_num=0 but
    applies for both returns.
    """

    def __init__(self,
                 field: str,
                 *,
                 info: Optional[core.SensorInfo] = None,
                 prefix: Optional[str] = "",
                 suffix: Optional[str] = "",
                 use_ae: bool = True,
                 use_buc: bool = False) -> None:
        """
        Args:
            info: sensor metadata used mainly for destaggering here
            field: name of field to process, second return is handled automatically
            prefix: name prefix
            suffix: name suffix
            use_ae: if True, use AutoExposure for the field
            use_buc: if True, use BeamUniformityCorrector for the field
        """
        self._info = info
        self._fields = [field]
        field2 = _second_chan_field(field)
        if field2:
            self._fields.append(field2)
        self._ae = _utils.AutoExposure() if use_ae else None
        self._buc = _utils.BeamUniformityCorrector() if use_buc else None
        self._prefix = f"{prefix}: " if prefix else ""
        self._suffix = f" ({suffix})" if suffix else ""
        self._wrap_name = lambda n: f"{self._prefix}{n}{self._suffix}"

    @property
    def name(self) -> str:
        return self._wrap_name(str(self._fields[0]))

    @property
    def names(self) -> List[str]:
        return [self._wrap_name(str(f)) for f in self._fields]

    def _prepare_data(self,
                      ls: core.LidarScan,
                      return_num: int = 0) -> Optional[np.ndarray]:
        if not self.enabled(ls, return_num):
            return None

        f = self._fields[return_num]
        field = ls.field(f)
        key_data = field if field.dtype == np.float32 else field.astype(
            np.float32)

        if self._buc:
            self._buc(key_data)

        if self._ae:
            self._ae(key_data, update_state=(return_num == 0))
        else:
            key_max = np.max(key_data)
            if key_max:
                key_data = key_data / key_max

        return key_data

    def set_image(self,
                  img: Image,
                  ls: core.LidarScan,
                  return_num: int = 0) -> None:
        if self._info is None:
            raise ValueError(
                f"VizMode[{self.name}] requires metadata to make a 2D image")
        key_data = self._prepare_data(ls, return_num)
        if key_data is not None:
            img.set_image(core.destagger(self._info, key_data))

    def set_cloud_color(self,
                        cloud: Cloud,
                        ls: core.LidarScan,
                        return_num: int = 0) -> None:
        key_data = self._prepare_data(ls, return_num)
        if key_data is not None:
            cloud.set_key(key_data)

    def enabled(self, ls: core.LidarScan, return_num: int = 0):
        return (self._fields[return_num] in ls.fields
                if return_num < len(self._fields) else False)


class RGBMode(ImageCloudMode):
    """RGB view mode
    """

    def __init__(self,
                 field: str,
                 *,
                 info: Optional[core.SensorInfo] = None) -> None:
        """
        Args:
            info: sensor metadata used mainly for destaggering here
            field: channel field to process
        """
        self._info = info
        self._field = field

    @property
    def name(self) -> str:
        return self._field

    @property
    def names(self) -> List[str]:
        return [self._field]

    def _prepare_data(self,
                      ls: core.LidarScan,
                      return_num: int = 0) -> Optional[np.ndarray]:

        field = ls.field(self._field)
        if np.ndim(field) != 3 and field.shape != 3:
            raise TypeError(f"Unsupport field shape: {field.shape}")
        if field.dtype == np.uint8:
            key_data = (field / (2**8 - 1)).astype(np.float32)
        elif field.dtype == np.uint16:
            key_data = (field / (2**16 - 1)).astype(np.float32)
        elif field.dtype == np.float32:
            key_data = field
        elif field.dtype == np.float64:
            key_data = field.astype(np.float32)
        else:
            raise TypeError(f"Unsupport field type {field.dtype}")

        return key_data.clip(0, 1.0)

    def set_image(self,
                  img: Image,
                  ls: core.LidarScan,
                  return_num: int = 0) -> None:
        if self._info is None:
            raise ValueError(
                f"VizMode[{self.name}] requires metadata to make a 2D image")
        key_data = self._prepare_data(ls)
        if key_data is not None:
            img.set_image(core.destagger(self._info, key_data))

    def set_cloud_color(self,
                        cloud: Cloud,
                        ls: core.LidarScan,
                        return_num: int = 0) -> None:
        key_data = self._prepare_data(ls)
        if key_data is not None:
            cloud.set_key(key_data)

    def enabled(self, ls: core.LidarScan, return_num: int = 0):
        field = ls.field(self._field)
        return np.ndim(field) == 3


class ReflMode(SimpleMode, ImageCloudMode):
    """Prepares image/cloud data for REFLECTIVITY channel"""

    def __init__(self, *, info: Optional[core.SensorInfo] = None) -> None:
        super().__init__(core.ChanField.REFLECTIVITY, info=info, use_ae=True)
        # used only for uncalibrated reflectivity in FW prior v2.1.0
        # TODO: should we check for calibrated reflectivity status from
        # metadata too?
        if self._info is not None:
            self._normalized_refl = (self._info.get_version() >=
                                     Version.from_string("v2.1.0"))
        else:
            # NOTE/TODO[pb]: ReflMode added through viz extra mode mechanism
            # may not have a correct normalized_refl set ... need a refactor.
            self._normalized_refl = True

    def _prepare_data(self,
                      ls: core.LidarScan,
                      return_num: int = 0) -> Optional[np.ndarray]:
        if not self.enabled(ls, return_num):
            return None

        f = self._fields[return_num]
        refl_data = ls.field(f).astype(np.float32)
        if self._normalized_refl:
            refl_data /= 255.0
        else:
            # mypy doesn't recognize that we always should have _ae here
            # so we have explicit check
            if self._ae:
                self._ae(refl_data, update_state=(return_num == 0))
        return refl_data


def is_norm_reflectivity_mode(mode: FieldViewMode) -> bool:
    """Checks whether the image/cloud mode is a normalized REFLECTIVITY mode
    """
    # NOTE[pb]: This is highly implementation specific and doesn't look nicely,
    # i.e. it's more like duck/duct plumbing .... but suits the need.
    return (isinstance(mode, ReflMode) and mode._normalized_refl)


LidarScanVizMode = Union[ImageCloudMode, ImageMode, CloudMode]
"""Field view mode types"""


@dataclass
class CloudPaletteItem:
    """Palette with a name"""
    name: str
    palette: np.ndarray
