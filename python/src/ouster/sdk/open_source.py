from typing import List, Optional, Union, Callable, Dict
import os
from ouster.sdk.core import FrameSetSource, PacketSource, OusterIoType, io_type
from ouster.sdk.osf import OsfFrameSetSource
from ouster.sdk.bag import BagFrameSetSource, BagPacketSource
from ouster.sdk._bindings.client import open_source as _open_source
from ouster.sdk._bindings.client import collate as _collate
from ouster.sdk._bindings.client import open_packet_source as _open_packet_source
from functools import partial


def _url_to_osf_url(url: str):
    if "http://studio.ouster.com" in url or "https://studio.ouster.com" in url:
        from ouster.sdk.util import studio
        new_url = studio.get_osf_from_data_app_link(url)
        assert new_url is not None
        return new_url
    else:
        return url


def _python_source_wrapper(ty: type, ioty: OusterIoType, source_url: Union[str, List[str]], collate: bool,
                           sensor_idx: int, *args, **kwargs) -> FrameSetSource:
    # handle multiple files
    src: FrameSetSource
    is_collated = False
    if ioty == OusterIoType.URL:
        # try and get the real url
        if isinstance(source_url, list):
            first_url = source_url[0]
        else:
            first_url = source_url
        new_url = _url_to_osf_url(first_url)
        src = OsfFrameSetSource(new_url)
        is_collated = src.is_collated
    else:
        if type(source_url) is list and len(source_url) > 1:
            for name in source_url:
                if io_type(name) != ioty:
                    raise RuntimeError("All sources must have the same type.")
        src = ty(source_url, *args, **kwargs)
    if sensor_idx >= 0:
        src = src.single(sensor_idx)
    elif collate and not is_collated:
        src = _collate(src)
    return src


io_type_handlers = {
    OusterIoType.BAG: partial(_python_source_wrapper, BagFrameSetSource, OusterIoType.BAG),
    OusterIoType.MCAP: partial(_python_source_wrapper, BagFrameSetSource, OusterIoType.MCAP),
    OusterIoType.URL: partial(_python_source_wrapper, type(None), OusterIoType.URL)
}

packet_io_type_handlers: Dict[OusterIoType, Callable[..., PacketSource]] = {
    OusterIoType.BAG: BagPacketSource,
    OusterIoType.MCAP: BagPacketSource,
}


class SourceURLException(Exception):
    def __init__(self, sub_exception, url, packet = False):
        self._sub_exception = sub_exception
        self._url = url
        self._packet = packet

    def __str__(self):
        if self._packet:
            result = f"Failed to create packet_source for url {self._url}\n"
        else:
            result = f"Failed to create frame_set_source for URL {self._url}\n"
        result += f"more details: {self._sub_exception}"
        return result

    def get_sub_exception(self):
        return self._sub_exception

    def get_url(self):
        return self._url


def open_source(source_url: Union[str, List[str]], collate: bool = True, sensor_idx: int = -1, *args,
                **kwargs) -> FrameSetSource:
    """Open a frame set source from one or more URLs.

    Args:
        source_url: A single URL or a list of URLs. Each URL can be a path to
            a pcap or osf file, or a sensor hostname/IP address.
        collate: If True, collate the source.
        sensor_idx: If >= 0, only output data from that specific sensor.
        extrinsic_file: Path to an extrinsics file.
        extrinsics: A single 4x4 numpy array or a list of 4x4 numpy arrays.
            When a single array is given and the frame set source has more than one
            sensor, the same extrinsics are applied to all sensors. Extra
            entries beyond the number of sensors are discarded.
        index: Index the source before starting if the format does not
            natively support indexing. Does not apply to live sources.
    """
    if len(source_url) == 0:
        raise ValueError("No valid source specified")

    if type(source_url) is list:
        source_url = [os.path.expanduser(url) for url in source_url]
    else:
        source_url = os.path.expanduser(source_url)  # type: ignore

    first_url: str
    first_url = source_url[0] if type(source_url) is list else source_url  # type: ignore

    source_type: OusterIoType
    frame_set_source: Optional[FrameSetSource] = None
    try:
        source_type = io_type(first_url)
        handler = _open_source
        if source_type in io_type_handlers:
            handler = io_type_handlers[source_type]
        frame_set_source = handler(source_url, collate=collate, sensor_idx=sensor_idx, *args, **kwargs)  # type: ignore
    except Exception as ex:
        raise SourceURLException(ex, source_url if type(source_url) is list else [source_url], False)  # type: ignore

    if frame_set_source is None:
        raise RuntimeError(
            f"Failed to create frame_set_source for URL {source_url}")

    return frame_set_source


def open_packet_source(source_url: Union[str, List[str]], *args,
                **kwargs) -> PacketSource:
    """Open a packet source from one or more URLs.

    Args:
        source_url: A single URL or a list of URLs. Each URL can be a path to
            a pcap or osf file, or a sensor hostname/IP address.
        extrinsics_file: Path to an extrinsics file.
        extrinsics: A single 4x4 numpy array or a list of 4x4 numpy arrays.
            When a single array is given and the frame set source has more than one
            sensor, the same extrinsics are applied to all sensors. Extra
            entries beyond the number of sensors are discarded.
        index: Index the source before starting if the format does not
            natively support indexing. Does not apply to live sources.
    """
    if len(source_url) == 0:
        raise ValueError("No valid source specified")

    if type(source_url) is list:
        source_url = [os.path.expanduser(url) for url in source_url]
    else:
        source_url = os.path.expanduser(source_url)  # type: ignore

    first_url: str
    first_url = source_url[0] if type(source_url) is list else source_url  # type: ignore

    source_type: OusterIoType
    frame_set_source: Optional[PacketSource] = None
    try:
        source_type = io_type(first_url)
        handler = _open_packet_source
        if source_type in io_type_handlers:
            handler = packet_io_type_handlers[source_type]
        frame_set_source = handler(source_url, *args, **kwargs)
    except Exception as ex:
        raise SourceURLException(ex, source_url if type(source_url) is list else [source_url], True)  # type: ignore

    if frame_set_source is None:
        raise RuntimeError(
            f"Failed to create packet_source for url {source_url}")

    return frame_set_source
