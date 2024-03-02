from typing import Optional, Union
from ouster.client import ScanSource, MultiScanSource
from ouster.cli.plugins.io_type import OusterIoType, io_type
from ouster.osf.multi import OsfScanSource
from ouster.sdkx.pcap_scan_source import PcapScanSource
from ouster.sdkx.sensor_scan_source import SensorScanSource


def open_source(source_url: str, sensor_idx: int = -1, *args, **kwargs
                ) -> Union[ScanSource, MultiScanSource]:
    """
    Parameters:
        - source_url: could be a single url or many for the case of sensors.
            the url can contain the path to a pcap file or an osf file.
            alternatively, the url could contain a list of comma separated
            sensor hostnames or ips (current multisensor is not supprted)
        - sensor_idx: If sensor_idx is set to a postive number the function
            returns a ScanSource instead of MultiScanSource which is simpler
            but can can handle one source. sensor_idx shouldn't exceed the
            number of scan sources that the source_url refers to.
    Other Common Parameters
        - cycle: loop when the stream ends, applies only to non-live sources.
        - index: index the source before start if the format doesn't natively have
            an index. doens't apply to live sources.
    """
    source_urls = [url.strip() for url in source_url.split(',') if url.strip()]

    if len(source_urls) == 0:
        raise ValueError("No valid source specificed")

    if len(source_urls) > 1:
        # This only applies to the live sensors case.
        # TODO: revise once working on multi sensors
        raise NotImplementedError(
            "providing more than a single url is current not supported!")

    # TODO: allow injecting/overriding with additional source type handlers
    io_type_handlers = {
        OusterIoType.SENSOR: SensorScanSource,
        OusterIoType.PCAP: PcapScanSource,
        OusterIoType.OSF: OsfScanSource
    }

    scan_source: Optional[MultiScanSource] = None
    try:
        source_type = io_type(source_urls[0])
        handler = io_type_handlers[source_type]
        scan_source = handler(source_urls[0], *args, **kwargs)
    except KeyError:
        raise NotImplementedError(f"The io_type:{source_type} is not supported!")
    except Exception as ex:
        raise RuntimeError(f"Failed to create scan_source for url {source_urls}\n"
                           f" more details: {ex}")

    if scan_source is None:
        raise RuntimeError(f"Failed to create scan_source for url {source_urls}")

    if sensor_idx < 0:
        return scan_source

    if sensor_idx < scan_source.sensors_count:
        # return the simplifed single stream interface
        return scan_source.single_source(sensor_idx)

    raise ValueError(f"source idx = {sensor_idx} value exceeds the number "
                     f"of available sensors = {scan_source.sensors_count} "
                     f"from the source {source_url}")
