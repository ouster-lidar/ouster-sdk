def read_pcap_open_packet_source(hostname: str,
                                 lidar_port: int = 7502,
                                 imu_port: int = 7503,
                                 n_seconds: int = 10,
                                 preview_packets: int = 5) -> None:
    """Read packets from a live sensor using ``open_packet_source``.
    Args:
        hostname: hostname or IP of the sensor
        lidar_port: UDP port used for lidar packets
        imu_port: UDP port used for IMU packets
        n_seconds: number of seconds to poll the sensor
        preview_packets: number of packets to print in detail
    """

    # [doc-stag-pcap-record-imports]
    from ouster.sdk import open_packet_source
    from ouster.sdk import sensor
    from contextlib import closing
    # [doc-etag-pcap-record-imports]
    import time
    from typing import cast
    timeout_sec = 1.0
    from ouster.sdk._bindings import client as _client
    # [doc-stag-pcap-record-setup]
    with closing(open_packet_source(hostname,
                                    lidar_port=lidar_port,
                                    imu_port=imu_port,
                                    buffer_time_sec=1.0)) as packet_source:
        # [doc-etag-pcap-record-setup]
        infos = packet_source.sensor_info
        if not infos:
            raise RuntimeError("No sensor metadata available")
        print(f"Connected to {hostname} with {len(infos)} sensor(s). Serial="
              f"{infos[0].sn}")

        if not isinstance(packet_source, sensor.SensorPacketSource):
            raise RuntimeError("Expected SensorPacketSource")

        lidar_packets = 0
        imu_packets = 0
        printed = 0

        deadline = time.time() + n_seconds
        while time.time() < deadline:
            event = cast(_client.ClientEvent, packet_source.get_packet(timeout_sec))

            if event.type == _client.ClientEventType.Exit:
                break

            if event.type != _client.ClientEventType.Packet:
                continue

            packet = cast(_client.Packet, event.packet())
            if packet.type == _client.PacketType.Lidar:
                lidar_packets += 1
            elif packet.type == _client.PacketType.Imu:
                imu_packets += 1

            if printed < preview_packets:
                print(
                    f"Packet {printed + 1}: "
                    f"type={packet.type.name} "
                    f"bytes={len(packet.buf)} "
                    f"host_ts={packet.host_timestamp}"
                )
                printed += 1

        print(f"Captured {lidar_packets} lidar packet(s) and "
              f"{imu_packets} imu packet(s) in {n_seconds} second(s).")
