from typing import (List, Optional, Union, Protocol, runtime_checkable, Tuple)

from dataclasses import dataclass
import numpy as np
from ouster.sdk import core
from ouster.sdk.core import (Version, AutoExposure, BeamUniformityCorrector,
                             LocalToneMapper)

from ouster.sdk._bindings.viz import Cloud, Image


@runtime_checkable
class FieldViewMode(Protocol):
    """LidarFrame field processor

    View modes define the process of getting the key data for
    the frame and return number as well as checks the possibility
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
                      ls: core.LidarFrame,
                      return_num: int = 0) -> Optional[np.ndarray]:
        """Prepares data for visualization given the frame and return number"""
        ...

    def enabled(self, ls: core.LidarFrame, return_num: int = 0) -> bool:
        """Checks the view mode availability for a frame and return number"""
        ...


@runtime_checkable
class ImageMode(FieldViewMode, Protocol):
    """Applies the view mode key to the viz.Image"""

    def set_image(self,
                  img: Image,
                  ls: core.LidarFrame,
                  return_num: int = 0) -> None:
        """Prepares the key data and sets the image key to it."""
        ...


@runtime_checkable
class CloudMode(FieldViewMode, Protocol):
    """Applies the view mode key to the viz.Cloud"""

    def set_cloud_color(self,
                        cloud: Cloud,
                        ls: core.LidarFrame,
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
        core.ChanField.FLAGS: core.ChanField.FLAGS2,
        core.ChanField.NORMALS: core.ChanField.NORMALS2,
        core.ChanField.GROUND: core.ChanField.GROUND2
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
        self._key_data: Optional[np.ndarray] = None

    @property
    def name(self) -> str:
        return "RING"

    @property
    def names(self) -> List[str]:
        return ["RING"]

    def _prepare_data(self,
                      ls: core.LidarFrame,
                      return_num: int = 0) -> Optional[np.ndarray]:
        if self._key_data is None:
            key_data = np.empty((self._info.h, self._info.w), dtype=np.uint8)
            for i in range(0, self._info.h):
                key_data[i, :] = int((i / self._info.h) * 255.0)
            self._key_data = key_data
        return self._key_data

    def set_cloud_color(self,
                        cloud: Cloud,
                        ls: core.LidarFrame,
                        return_num: int = 0) -> None:
        self._prepare_data(ls, return_num)
        assert self._key_data is not None
        cloud.set_key(self._key_data)

    def enabled(self, ls: core.LidarFrame, return_num: int = 0):
        return True


class SensorMode(CloudMode):
    """View mode to show sensor index."""

    def __init__(self, info: core.SensorInfo, color: Tuple[int, int, int]) -> None:
        """
        Args:
            info: sensor metadata
        """
        self._info = info
        self._color = color
        self._key_data: Optional[np.ndarray] = None

    @property
    def name(self) -> str:
        return "SENSOR"

    @property
    def names(self) -> List[str]:
        return ["SENSOR"]

    def _prepare_data(self,
                      ls: core.LidarFrame,
                      return_num: int = 0) -> Optional[np.ndarray]:
        if self._key_data is None:
            self._key_data = np.empty((self._info.h, self._info.w, 3), dtype=np.uint8)
            self._key_data[:] = self._color
        return self._key_data

    def set_cloud_color(self,
                        cloud: Cloud,
                        ls: core.LidarFrame,
                        return_num: int = 0) -> None:
        self._prepare_data(ls, return_num)
        assert self._key_data is not None
        cloud.set_key(self._key_data)

    def enabled(self, ls: core.LidarFrame, return_num: int = 0):
        return True


class TimestampMode(CloudMode):
    """View mode to show column timestamp."""

    def __init__(self, info: core.SensorInfo) -> None:
        """
        Args:
            info: sensor metadata
        """
        self._info = info

    @property
    def name(self) -> str:
        return "TIMESTAMP"

    @property
    def names(self) -> List[str]:
        return ["TIMESTAMP"]

    def _prepare_data(self,
                      ls: core.LidarFrame,
                      return_num: int = 0) -> Optional[np.ndarray]:
        nonzero = np.nonzero(ls.status)
        min = np.min(ls.timestamp[nonzero])
        timestamps = (ls.timestamp - min).astype(np.float32, copy=True)
        delta = np.max(ls.timestamp) - min
        # handle case when all points have same value to avoid divide by zero
        if delta <= 0:
            delta = 1.0
        timestamps /= delta
        key_data = np.tile(timestamps, (ls.h, 1))
        return key_data

    def set_cloud_color(self,
                        cloud: Cloud,
                        ls: core.LidarFrame,
                        return_num: int = 0) -> None:
        key_data = self._prepare_data(ls, return_num)
        if key_data is not None:
            cloud.set_key(key_data)

    def enabled(self, ls: core.LidarFrame, return_num: int = 0):
        return True


class SimpleMode(ImageCloudMode):
    """Basic view mode with AutoExposure and BeamUniformityCorrector

    Handles single and dual returns frames.

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
                 use_buc: bool = False,
                 scale: Optional[float] = None) -> None:
        """
        Args:
            info: sensor metadata used mainly for destaggering here
            field: name of field to process, second return is handled automatically
            prefix: name prefix
            suffix: name suffix
            use_ae: if True, use AutoExposure for the field
            use_buc: if True, use BeamUniformityCorrector for the field
            scale: if use_ae is false and this is set, use this to scale the values for display
        """
        self._info = info
        self._fields = [field]
        field2 = _second_chan_field(field)
        if field2:
            self._fields.append(field2)
        self._ae = AutoExposure() if use_ae else None
        self._buc = BeamUniformityCorrector() if use_buc else None
        self._prefix = f"{prefix}: " if prefix else ""
        self._suffix = f" ({suffix})" if suffix else ""
        self._wrap_name = lambda n: f"{self._prefix}{n}{self._suffix}"
        self._scale = scale

    @property
    def name(self) -> str:
        return self._wrap_name(str(self._fields[0]))

    @property
    def names(self) -> List[str]:
        return [self._wrap_name(str(f)) for f in self._fields]

    def _prepare_data(self,
                      ls: core.LidarFrame,
                      return_num: int = 0) -> Optional[np.ndarray]:
        if not self.enabled(ls, return_num):
            return None

        f = self._fields[return_num]
        field = ls.field(f)
        key_data = field.astype(np.float32, copy=True)

        if self._buc:
            self._buc.update(key_data)

        if self._ae:
            self._ae.update(key_data, update_state=(return_num == 0))
        elif self._scale is not None:
            key_data *= self._scale
        else:
            key_max = np.max(key_data)
            if key_max:
                key_data = key_data / key_max

        return key_data

    def set_image(self,
                  img: Image,
                  ls: core.LidarFrame,
                  return_num: int = 0) -> None:
        if self._info is None:
            raise ValueError(
                f"VizMode[{self.name}] requires metadata to make a 2D image")
        key_data = self._prepare_data(ls, return_num)
        if key_data is not None:
            img.set_image(core.destagger(self._info, key_data))

    def set_cloud_color(self,
                        cloud: Cloud,
                        ls: core.LidarFrame,
                        return_num: int = 0) -> None:
        key_data = self._prepare_data(ls, return_num)
        if key_data is not None:
            cloud.set_key(key_data)

    def enabled(self, ls: core.LidarFrame, return_num: int = 0):
        return (self._fields[return_num] in ls.fields
                if return_num < len(self._fields) else False)


class RGBMode(ImageCloudMode):
    """RGB view mode"""

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
                      ls: core.LidarFrame,
                      return_num: int = 0) -> Optional[np.ndarray]:

        field = ls.field(self._field)
        if np.ndim(field) != 3 and field.shape != 3:
            raise TypeError(f"Unsupport field shape: {field.shape}")
        if field.dtype == np.uint8:
            return field
        elif field.dtype == np.uint16:
            return (field >> 8).astype(np.uint8)
        elif field.dtype == np.float32:
            key_data = field
        elif field.dtype == np.float64:
            key_data = field.astype(np.float32, copy=True)
        else:
            raise TypeError(f"Unsupport field type {field.dtype}")

        return key_data.clip(0, 1.0)

    def set_image(self,
                  img: Image,
                  ls: core.LidarFrame,
                  return_num: int = 0) -> None:
        if self._info is None:
            raise ValueError(
                f"VizMode[{self.name}] requires metadata to make a 2D image")
        key_data = self._prepare_data(ls)
        if key_data is not None:
            img.set_image(core.destagger(self._info, key_data))

    def set_cloud_color(self,
                        cloud: Cloud,
                        ls: core.LidarFrame,
                        return_num: int = 0) -> None:
        key_data = self._prepare_data(ls)
        if key_data is not None:
            cloud.set_key(key_data)

    def enabled(self, ls: core.LidarFrame, return_num: int = 0):
        field = ls.field(self._field)
        return np.ndim(field) == 3


class HDRRGBMode(ImageCloudMode):
    """RGB view mode using LocalToneMapper."""

    def __init__(self,
                 field: str,
                 info: core.SensorInfo) -> None:
        self._info = info
        self._field = field
        self._tonemapper = LocalToneMapper()
        self._last_frame: Optional[core.LidarFrame] = None
        self._last_data: Optional[np.ndarray] = None

    @property
    def name(self) -> str:
        return self._field

    @property
    def names(self) -> List[str]:
        return [self._field]

    def _prepare_data(self,
                      ls: core.LidarFrame,
                      return_num: int = 0) -> Optional[np.ndarray]:
        if not self.enabled(ls, return_num):
            return None
        if ls is self._last_frame:
            return self._last_data
        self._last_frame = ls

        field = ls.field(self._field)
        if field.dtype != np.float16:
            raise TypeError(f"Unsupported field type: {field.dtype}")

        f16_destag = core.destagger(self._info, field)
        sdr_destag = self._tonemapper.update(f16_destag)
        sdr_data = core.stagger(self._info, sdr_destag)
        self._last_data = sdr_data
        self._last_destag_data = sdr_destag
        return sdr_data

    def set_image(self,
                  img: Image,
                  ls: core.LidarFrame,
                  return_num: int = 0) -> None:
        if self._info is None:
            raise ValueError(
                f"VizMode[{self.name}] requires metadata to make a 2D image")
        key_data = self._prepare_data(ls, return_num)
        if key_data is not None:
            img.set_image(self._last_destag_data)

    def set_cloud_color(self,
                        cloud: Cloud,
                        ls: core.LidarFrame,
                        return_num: int = 0) -> None:
        key_data = self._prepare_data(ls, return_num)
        if key_data is not None:
            cloud.set_key(key_data)

    def enabled(self, ls: core.LidarFrame, return_num: int = 0) -> bool:
        field = ls.field(self._field)
        return np.ndim(field) == 3 and field.shape[2] == 3


class NormalsMode(ImageCloudMode):
    """Normals value remap [-1, 1] -> [0, 1]"""

    def __init__(self,
                 field: str,
                 *,
                 info: Optional[core.SensorInfo] = None) -> None:
        self._info = info
        self._fields = [field]
        field2 = _second_chan_field(field)
        if field2:
            self._fields.append(field2)

    @property
    def name(self) -> str:
        return str(self._fields[0])

    @property
    def names(self) -> List[str]:
        return [str(field) for field in self._fields]

    def _prepare_data(self,
                      ls: core.LidarFrame,
                      return_num: int = 0) -> Optional[np.ndarray]:
        if not self.enabled(ls, return_num):
            return None

        field = self._fields[return_num]
        data = ls.field(field)
        if np.ndim(data) != 3 or data.shape[-1] != 3:
            raise TypeError(f"Unsupported normal field shape: {data.shape}")
        key_data = np.asarray(data, dtype=np.float32)
        zero_mask = np.all(key_data == 0.0, axis=-1)
        np.clip(key_data, -1.0, 1.0, out=key_data)
        key_data = 0.5 * (key_data + 1.0)
        if np.any(zero_mask):
            key_data[zero_mask] = 0.0
        return key_data

    def set_image(self,
                  img: Image,
                  ls: core.LidarFrame,
                  return_num: int = 0) -> None:
        if self._info is None:
            raise ValueError(
                f"VizMode[{self.name}] requires metadata to make a 2D image")
        key_data = self._prepare_data(ls, return_num)
        if key_data is not None:
            img.set_image(core.destagger(self._info, key_data))

    def set_cloud_color(self,
                        cloud: Cloud,
                        ls: core.LidarFrame,
                        return_num: int = 0) -> None:
        key_data = self._prepare_data(ls, return_num)
        if key_data is not None:
            cloud.set_key(key_data)

    def enabled(self, ls: core.LidarFrame, return_num: int = 0):
        if return_num >= len(self._fields):
            return False

        field = self._fields[return_num]
        if field not in ls.fields:
            return False

        data = ls.field(field)
        return np.ndim(data) == 3 and data.shape[-1] == 3


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
                      ls: core.LidarFrame,
                      return_num: int = 0) -> Optional[np.ndarray]:
        if not self.enabled(ls, return_num):
            return None

        f = self._fields[return_num]
        refl_data = ls.field(f).astype(np.float32, copy=True)
        if self._normalized_refl:
            refl_data /= 255.0
        else:
            # mypy doesn't recognize that we always should have _ae here
            # so we have explicit check
            if self._ae:
                self._ae.update(refl_data, update_state=(return_num == 0))
        return refl_data


def is_norm_reflectivity_mode(mode: FieldViewMode) -> bool:
    """Checks whether the image/cloud mode is a normalized REFLECTIVITY mode
    """
    # NOTE[pb]: This is highly implementation specific and doesn't look nicely,
    # i.e. it's more like duck/duct plumbing .... but suits the need.
    return (isinstance(mode, ReflMode) and mode._normalized_refl)


LidarFrameVizMode = Union[ImageCloudMode, ImageMode, CloudMode]
"""Field view mode types"""


@dataclass
class CloudPaletteItem:
    """Palette with a name"""
    name: str
    palette: np.ndarray
