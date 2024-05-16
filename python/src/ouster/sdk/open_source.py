from typing import List, Optional, Union
import numpy as np
from pathlib import Path
from ouster.sdk.client import ScanSource, MultiScanSource
import ouster.sdk.io_type
from ouster.sdk.io_type import OusterIoType
from ouster.sdk.osf import OsfScanSource
from ouster.sdk.pcap import PcapScanSource
from ouster.sdk.sensor import SensorScanSource


io_type_handlers = {
    OusterIoType.SENSOR: SensorScanSource,
    OusterIoType.PCAP: PcapScanSource,
    OusterIoType.OSF: OsfScanSource
}


def open_source(source_url: str, sensor_idx: int = 0, *args,
                extrinsics: Optional[Union[str, np.ndarray, List[np.ndarray]]] = None,
                **kwargs) -> Union[ScanSource, MultiScanSource]:
    """
    Parameters:
        - source_url: could be a single url or many for the case of sensors.
            the url can contain the path to a pcap file or an osf file.
            alternatively, the url could contain a list of comma separated
            sensor hostnames or ips (current multisensor is not supprted)
        - sensor_idx: The sensor_idx parameter can be set to any value between
            0 and the number of addressable sensors within the source_url. This
            returns the ScanSource interface (default is 0). If you want to work
            with the more advanced MultiScanSource interface which supports
            handling multiple sensors at the same time pass -1.
        - extrinsics: could be either a path to an extrinsics file, single 4x4
            numpy array or a list of 4x4 numpy arrays. In case a single 4x4 numpy
            array was given while the scan_source had more than sensor then the
            same extrinsics is copied and applied to all the sensors. If the list
            of provided extrinsics exceeds the number of available sensors in
            the scan source then the extra will be discarded.
    Other Common Parameters
        - cycle: loop when the stream ends, applies only to non-live sources.
        - index: index the source before start if the format doesn't natively have
            an index. doens't apply to live sources.
        - flags: when this option is set, the FLAGS field will be added to the list
            of fields of every scan. in case of dual returns profile FLAGS2 will also
            be appended (default is True).
    """
    source_urls = [url.strip() for url in source_url.split(',') if url.strip()]

    if len(source_urls) == 0:
        raise ValueError("No valid source specified")

    if len(source_urls) > 1:
        # This only applies to the live sensors case.
        # TODO: revise once working on multi sensors
        raise NotImplementedError(
            "providing more than a single url is current not supported!")

    source_type: OusterIoType
    scan_source: Optional[MultiScanSource] = None
    try:
        source_type = ouster.sdk.io_type.io_type(source_urls[0])
        handler = io_type_handlers[source_type]
        scan_source = handler(source_urls[0], *args, **kwargs)
    except KeyError:
        raise NotImplementedError(
            f"The io_type:{source_type} is not supported!")
    except Exception as ex:
        raise RuntimeError(f"Failed to create scan_source for url {source_urls}\n"
                           f" more details: {ex}")

    if scan_source is None:
        raise RuntimeError(
            f"Failed to create scan_source for url {source_urls}")

    _populate_extrinsics(scan_source, source_urls[0], source_type, extrinsics)

    if sensor_idx < 0:
        return scan_source

    if sensor_idx < scan_source.sensors_count:
        # return the simplifed single stream interface
        return scan_source.single_source(sensor_idx)

    raise ValueError(f"source idx = {sensor_idx} value exceeds the number "
                     f"of available sensors = {scan_source.sensors_count} "
                     f"from the source {source_url}")


def _populate_extrinsics(scan_source: MultiScanSource,
                         source_url: str,
                         source_type: OusterIoType,
                         _extrinsics: Optional[Union[str, np.ndarray, List[np.ndarray]]] = None) -> None:

    from ouster.sdk.util import resolve_extrinsics, _parse_extrinsics_file

    extrinsics: Optional[List[np.ndarray]] = None

    if _extrinsics is not None:
        # handle single numpy array case
        if isinstance(_extrinsics, np.ndarray) and _extrinsics.shape == (4, 4):
            extrinsics = [_extrinsics] * scan_source.sensors_count
        # handle list of numpy array case
        elif isinstance(_extrinsics, list) and all(
            [isinstance(ext, np.ndarray) and ext.shape == (4, 4)
             for ext in _extrinsics]):
            extrinsics = _extrinsics
        # handle extrinsics as a file path
        elif isinstance(_extrinsics, str):
            sensors_serial = [info.sn for info in scan_source.metadata]
            extrinsics_from_file = _parse_extrinsics_file(
                _extrinsics, sensors_serial)
            if extrinsics_from_file:
                extrinsics = [ext_f[0]
                              for ext_f in extrinsics_from_file if ext_f]
            else:
                print(f"warning: failed to load extrinsics from provided path: "
                      f"{_extrinsics}")
        else:
            raise ValueError(
                f"Error whiles parsing supplied extrinsics {_extrinsics}")
    else:
        if source_type in [OusterIoType.PCAP, OusterIoType.OSF]:
            source_dir = Path(source_url).absolute().parent
            # print(f"examining the directory '{source_dir}' for any extrinsics")
            extrinsics_from_file = resolve_extrinsics(data_path=source_dir,
                                                      infos=scan_source.metadata)
            if extrinsics_from_file:
                extrinsics = [ext_f[0]
                              for ext_f in extrinsics_from_file if ext_f]

    if extrinsics:
        if len(extrinsics) < scan_source.sensors_count:
            # TODO: should we handle the case when sensor_idx >= 0
            print("warning: loaded externsics doesn't match to the count of"
                  f"sensors. provided {len(extrinsics)}, expected: {scan_source.sensors_count}")
        for i in range(scan_source.sensors_count):
            if extrinsics[i] is not None:
                scan_source.metadata[i].extrinsic = extrinsics[i]
