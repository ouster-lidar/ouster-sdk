import sys

# [doc-stag-zone-state-imports]
from ouster.sdk import open_source
from ouster.sdk import core, sensor
# [doc-etag-zone-state-imports]

if __name__ == "__main__":
    hostname = sys.argv[1]
    destination_ip = sys.argv[2]

    # [doc-stag-set-live-zones]
    http = sensor.SensorHttp.create(hostname)
    print("Setting live zones to {0, 1, 2, 3}...")
    http.set_zone_monitor_live_ids([0, 1, 2, 3])
    # [doc-etag-set-live-zones]

    # [doc-stag-set-zm-udp-dest]
    config = core.SensorConfig()
    config.udp_dest_zm = destination_ip
    config.udp_port_zm = 7504
    print(f"Setting Zone Monitor UDP destination to "
          f"{destination_ip}:"
          "7504...")
    sensor.set_config(hostname, config)
    # [doc-etag-set-zm-udp-dest]

    # [doc-stag-read-zone-states]
    print("Connecting to sensor and reading zone states...")
    source = open_source(hostname)
    for frame_set in source:
        if len(frame_set) > 0 and frame_set[0] is not None:
            frame = frame_set[0]
            if frame.has_field("ZONE_STATES"):
                zone_states = frame.zones
                print(f"Frame ID: {frame.frame_id},"
                      "Zone States: [", end="")
                first = True
                for zone in zone_states:
                    if zone.live:
                        if not first:
                            print(", ", end="")
                        print(
                             f"({int(zone.id)}, "
                             f"{int(zone.trigger_status)})", end="")
                        first = False

                print("]")

    # [doc-etag-read-zone-states]
