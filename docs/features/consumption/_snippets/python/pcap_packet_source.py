# Copyright (c) 2024, Ouster, Inc.
# All rights reserved.
"""Demonstrate reading typed packets from a PCAP file using PcapPacketSource."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Union

from ouster.sdk import pcap, core


def count_packets(
    pcap_path: Union[str, Path],
) -> Dict[str, int]:
    """Count lidar and IMU packets in a PCAP file.

    Returns:
        Dict with 'lidar' and 'imu' counts.
    """
    counts: Dict[str, int] = {"lidar": 0, "imu": 0}

    # [doc-stag-pcap-packet-source-python]
    source = pcap.PcapPacketSource(str(pcap_path))
    metadata = source.sensor_info[0]
    packet_format = core.PacketFormat(metadata)

    for sensor_idx, packet in source:
        if isinstance(packet, core.LidarPacket):
            frame_id = packet_format.frame_id(packet.buf)
            print(f"sensor={sensor_idx} lidar frame_id={frame_id}"
                  f" bytes={len(packet.buf)}")
            counts["lidar"] += 1
        elif isinstance(packet, core.ImuPacket):
            print(f"sensor={sensor_idx} imu"
                  f" host_ts={packet.host_timestamp}")
            counts["imu"] += 1
    source.close()
    # [doc-etag-pcap-packet-source-python]

    return counts


def main(pcap_file: str) -> None:
    """CLI entry point."""
    result = count_packets(pcap_file)
    print(f"Total: {result['lidar']} lidar packet(s), {result['imu']} imu packet(s)")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Count packets in a PCAP file using PcapPacketSource."
    )
    parser.add_argument("pcap_file", help="Path to the PCAP file.")
    args = parser.parse_args()
    main(args.pcap_file)
