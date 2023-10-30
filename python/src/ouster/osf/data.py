from ouster import client
import ouster.osf as osf

import numpy as np
from typing import cast, Iterator, Union, Tuple, List


class Scans(client.ScanSource):
    """An iterable stream of ``LidarScan`` read from OSF file (for the first available sensor)."""

    def __init__(self,
                 osf_file: str,
                 *,
                 cycle: bool = False,
                 start_ts: int = 0,
                 sensor_id: int = 0):
        """
        Args:
            osf_file: OSF filename as scans source
            cycle: repeat infinitely after iteration is finished is True
            start_ts: return lidar scans starting from the specified start_ts
                      (in nanoseconds)
            sensor_id: id of the sensor which LidarScan stream data to read
            (i.e. id of the metadata entry with ``osf.LidarSensor`` type).
            0 (default) means that first LidarSensor from the OSF is used.
        """
        self._reader = osf.Reader(osf_file)
        self._cycle = cycle
        self._start_ts = start_ts
        self._sensor_id = sensor_id

        if self._sensor_id:
            # sensor_id is passed so we can get the sensor metadata
            # entry directly by metadata entry id
            sensor_meta = self._reader.meta_store[self._sensor_id]
            if sensor_meta and sensor_meta.of(osf.LidarSensor):
                self._sensor = sensor_meta
            else:
                raise ValueError(f"Error: Sensor is not found by sensor_id: "
                                 f" {self._sensor_id}")
        else:
            # sensor_id is not provided, so we get the first
            # osf.LidarSensor metadata entry and use its stream
            sensor_meta = self._reader.meta_store.get(osf.LidarSensor)
            if not sensor_meta:
                raise ValueError("Error: No sensors found in OSF file")
            self._sensor = sensor_meta

        # check for Extrinsics
        extrinsics = self._reader.meta_store.find(osf.Extrinsics)
        for _, v in extrinsics.items():
            if v.ref_meta_id == self._sensor.id:
                print(f"Found extrinsics for sensor[{self._sensor.id}]:\n",
                      v.extrinsics)
                self._sensor.info.extrinsic = v.extrinsics

        # Find the corresponding stream_id for the sensor
        scan_streams = self._reader.meta_store.find(osf.LidarScanStream)
        self._sensor_stream_id = next((mid for mid, m in scan_streams.items()
                                       if m.sensor_meta_id == self._sensor.id),
                                      0)
        if not self._sensor_stream_id:
            raise ValueError(f"Error: No LidarScan stream found for sensor"
                             f" id:{self._sensor.id} in an OSF file")

    def __iter__(self) -> Iterator[client.LidarScan]:
        """Iterator that returns ``LidarScan`` objects."""
        for _, ls in self.withTs():
            yield ls

    def withTs(self) -> Iterator[Tuple[int, client.LidarScan]]:
        """Iterator that returns tuple of (``ts``, ``LidarScan``)

        Where ``ts`` - is a timestamp (ns) of a ``LidarScan`` (usually as a
        timestamp of a first packet in a ``LidarScan``)
        """
        while True:
            # TODO[pb]: Read only specified _sensor_stream_id stream
            for msg in self._reader.messages([self._sensor_stream_id],
                                             self._start_ts,
                                             self._reader.end_ts):
                if msg.id == self._sensor_stream_id:
                    scan = msg.decode()
                    if scan:
                        yield msg.ts, cast(client.LidarScan, scan)
            if not self._cycle:
                break

    def close(self) -> None:
        # TODO[pb]: Do the close for Reader?
        pass

    @property
    def metadata(self) -> client.SensorInfo:
        """Return metadata of a Lidar Sensor used."""
        return self._sensor.info


def resolve_field_types(
    metadata: Union[client.SensorInfo, List[client.SensorInfo]],
    flags: bool = False,
    raw_headers: bool = False,
    raw_fields: bool = False
) -> Union[client.FieldTypes, List[client.FieldTypes]]:
    """Resolving optimal field types for OSF LidarScanStream encoder

    Shrinks the sizes of the LEGACY UDPLidarProfile fields and extends with
    FLAGS/FLAGS2 if `flags=True`.

    Args:
        metadata: single SensorInfo or a list of SensorInfo used resolve
                  UDPLidarProfile
        flags: True if augment the resulting fields with FLAGS/FLAGS2
        raw_headers: True if RAW_HEADERS field should be included (i.e. all
                     lidar packet headers and footers will be added during
                     batching)
        raw_fields: True if RAW32_WORDx fields should be included

    Returns:
        field types of a typical LidarScan with a requested optional fields.
    """

    single_result = False
    if not isinstance(metadata, list):
        metadata = [metadata]
        single_result = True

    field_types = []

    for i, m in enumerate(metadata):
        ftypes = client.get_field_types(m)
        profile = m.format.udp_profile_lidar

        # HACK: Overwrite fields to reduced datatypes for LEGACY (saves ~15% of
        # space in a file)
        if profile == client.UDPProfileLidar.PROFILE_LIDAR_LEGACY:
            ftypes.update(
                dict({
                    client.ChanField.RANGE: np.uint32,
                    client.ChanField.SIGNAL: np.uint16,
                    client.ChanField.REFLECTIVITY: np.uint16,
                    client.ChanField.NEAR_IR: np.uint16
                }))

        if flags:
            ftypes.update({client.ChanField.FLAGS: np.uint8})
            if client.ChanField.RANGE2 in ftypes:
                ftypes.update({client.ChanField.FLAGS2: np.uint8})

        if raw_fields:
            ftypes.update({client.ChanField.RAW32_WORD1: np.uint32})
            if profile != client.UDPProfileLidar.PROFILE_LIDAR_RNG15_RFL8_NIR8:
                # not Low Bandwidth
                ftypes.update(
                    {client.ChanField.RAW32_WORD2: np.uint32})
                ftypes.update(
                    {client.ChanField.RAW32_WORD3: np.uint32})
            if client.ChanField.RANGE2 in ftypes:
                ftypes.update(
                    {client.ChanField.RAW32_WORD4: np.uint32})
            if profile == client.UDPProfileLidar.PROFILE_LIDAR_FIVE_WORD_PIXEL:
                ftypes.update(
                    dict({
                        client.ChanField.RAW32_WORD4: np.uint32,
                        client.ChanField.RAW32_WORD5: np.uint32
                    }))

        if raw_headers:
            # getting the optimal field type for RAW_HEADERS
            pf = client._client.PacketFormat.from_info(m)
            h = pf.pixels_per_column
            raw_headers_space = (pf.packet_header_size +
                                 pf.packet_footer_size + pf.col_header_size +
                                 pf.col_footer_size)
            dtype = [
                np.uint8,
                np.uint16,
                np.uint32
            ][int(raw_headers_space / h)]
            ftypes.update({client.ChanField.RAW_HEADERS: dtype})  # type: ignore

        field_types.append(ftypes)

    return field_types[0] if single_result else field_types
