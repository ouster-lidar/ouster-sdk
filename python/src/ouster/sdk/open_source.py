from typing import List, Optional, Union, Callable, Dict
import os
from ouster.sdk.core import ScanSource, PacketSource, OusterIoType
import ouster.sdk.core.io_types
from ouster.sdk.bag import BagScanSource, BagPacketSource
from ouster.sdk._bindings.client import open_source as _open_source
from ouster.sdk._bindings.client import collate as _collate
from ouster.sdk._bindings.client import open_packet_source as _open_packet_source
from functools import partial


def _python_source_wrapper(ty: type, source_url: Union[str, List[str]], collate: bool,
                           sensor_idx: int, *args, **kwargs):
    src = ty(source_url, *args, **kwargs)
    if sensor_idx >= 0:
        src = src.single(sensor_idx)
    elif collate:
        src = _collate(src)
    return src


io_type_handlers = {
    OusterIoType.SENSOR: _open_source,
    OusterIoType.PCAP: _open_source,
    OusterIoType.OSF: _open_source,
    OusterIoType.BAG: partial(_python_source_wrapper, BagScanSource),
    OusterIoType.MCAP: partial(_python_source_wrapper, BagScanSource)
}

packet_io_type_handlers: Dict[OusterIoType, Callable[..., PacketSource]] = {
    OusterIoType.SENSOR: _open_packet_source,
    OusterIoType.PCAP: _open_packet_source,
    OusterIoType.OSF: _open_packet_source,
    OusterIoType.BAG: BagPacketSource,
    OusterIoType.MCAP: BagPacketSource,
}


class SourceURLException(Exception):
    def __init__(self, sub_exception, url):
        self._sub_exception = sub_exception
        self._url = url

    def __str__(self):
        result = f"Failed to create scan_source for url {self._url}\n"
        result += f"more details: {self._sub_exception}"
        return result

    def get_sub_exception(self):
        return self._sub_exception

    def get_url(self):
        return self._url


def open_source(source_url: Union[str, List[str]], collate: bool = True, sensor_idx: int = -1, *args,
                **kwargs) -> ScanSource:
    """
    Parameters:
        - source_url: could be a single url or many for the case of sensors.
            the url can contain the path to a pcap file or an osf file.
            alternatively, the argument could contain a list of sensor hostnames
            or ips
        - extrinsic_file: a path to an extrinsics file
        - extrinsics: single 4x4 numpy array or a list of 4x4 numpy arrays.
            In case a single 4x4 numpy
            array was given while the scan_source had more than sensor then the
            same extrinsics is copied and applied to all the sensors. If the list
            of provided extrinsics exceeds the number of available sensors in
            the scan source then the extra will be discarded.
        - collate: if true collate the source
        - sensor_idx: if >= 0 only output data from that specific sensor
    Other Common Parameters
        - index: index the source before start if the format doesn't natively have
            an index. doens't apply to live sources.
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
    scan_source: Optional[ScanSource] = None
    try:
        source_type = ouster.sdk.core.io_types.io_type(first_url)
        handler = io_type_handlers[source_type]
        scan_source = handler(source_url, collate=collate, sensor_idx=sensor_idx, *args, **kwargs)  # type: ignore
    except KeyError:
        raise NotImplementedError(
            f"The io_type:{source_type} is not supported!")
    except Exception as ex:
        raise SourceURLException(ex, source_url if type(source_url) is list else [source_url])  # type: ignore

    if scan_source is None:
        raise RuntimeError(
            f"Failed to create scan_source for url {source_url}")

    return scan_source


def open_packet_source(source_url: Union[str, List[str]], *args,
                **kwargs) -> PacketSource:
    """
    Parameters:
        - source_url: could be a single url or many for the case of sensors.
            the url can contain the path to a pcap file or an osf file.
            alternatively, the argument could contain a list of sensor hostnames
            or ips
        - extrinsics_file: a path to an extrinsics file
        - extrinsics: single 4x4 numpy array or a list of 4x4 numpy arrays.
            In case a single 4x4 numpy
            array was given while the scan_source had more than sensor then the
            same extrinsics is copied and applied to all the sensors. If the list
            of provided extrinsics exceeds the number of available sensors in
            the scan source then the extra will be discarded.
    Other Common Parameters
        - index: index the source before start if the format doesn't natively have
            an index. doens't apply to live sources.
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
    scan_source: Optional[PacketSource] = None
    try:
        source_type = ouster.sdk.core.io_types.io_type(first_url)
        handler = packet_io_type_handlers[source_type]
        scan_source = handler(source_url, *args, **kwargs)
    except KeyError:
        raise NotImplementedError(
            f"The io_type:{source_type} is not supported!")
    except Exception as ex:
        raise SourceURLException(ex, source_url if type(source_url) is list else [source_url])  # type: ignore

    if scan_source is None:
        raise RuntimeError(
            f"Failed to create packet_source for url {source_url}")

    return scan_source
