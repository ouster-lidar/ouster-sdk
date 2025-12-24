import sys
from ouster.sdk import open_source
from ouster.sdk.core import SensorConfig

if __name__ == "__main__":
    hostname = sys.argv[1]
    destination_ip = sys.argv[2]
    # [doc-stag-set-live-zones]
    from ouster.sdk.sensor import SensorHttp
    http = SensorHttp.create(hostname)
    http.set_zone_monitor_live_ids([0, 1, 2, 3])
    # [doc-etag-set-live-zones]

    # [doc-stag-set-zm-udp-dest]
    from ouster.sdk.sensor import set_config
    config = SensorConfig()
    config.udp_dest_zm = destination_ip
    config.udp_port_zm = 7504
    set_config(hostname, config)
    # [doc-etag-set-zm-udp-dest]

    # [doc-stag-read-zone-states]
    source = open_source(hostname)
    for scan, in source:
        if scan is not None:
            zone_states = scan.field('ZONE_STATES')
            print([(zone.id, zone.trigger_status) for zone in zone_states if zone.live])
    # [doc-etag-read-zone-states]
