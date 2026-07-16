import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Zone Monitor Emulation Example"
    )
    parser.add_argument("source", type=str,
                        help="Path to a recorded pcap file or sensor hostname/IP")
    parser.add_argument("zone_set", type=str, nargs='?',
                        help="Path to a zone set zip file")
    args = parser.parse_args()

    # [doc-stag-zone-monitor-emulation]
    import numpy as np
    from ouster.sdk import open_source
    from ouster.sdk.core import FieldClass
    from ouster.sdk.core import EmulatedZoneMon, ZoneSet
    source = open_source(args.source)
    if not args.zone_set:
        zone_set = source.sensor_info[0].zone_set
        assert zone_set is not None, "Sensor does not have a ZoneSet configured"
    else:
        zone_set = ZoneSet(args.zone_set)
        zone_set.render(source.sensor_info[0])

    emulator = EmulatedZoneMon(zone_set)
    emulator.live_zones = [0, 1, 2, 3]

    # process frames and emulate zone monitor triggers
    for frame, in source:

        if frame is None:
            continue

        # remove any existing zone monitor fields
        if frame.has_field('ZONE_STATES'):
            frame.del_field('ZONE_STATES')
        if frame.has_field('ZONE_OCCUPANCY'):
            frame.del_field('ZONE_OCCUPANCY')
        if frame.has_field('ZONE_PACKET_TIMESTAMP'):
            frame.del_field('ZONE_PACKET_TIMESTAMP')

        # update the emulation state with the current frame
        frame.add_field('ZONE_OCCUPANCY', np.uint16)
        emulator.calc_triggers(frame.field('RANGE'), frame.field('ZONE_OCCUPANCY'))
        zone_states = emulator.get_packet()

        # optionally, add the emulated zone monitor state back to the frame for further processing
        frame.add_field('ZONE_STATES', zone_states, FieldClass.FRAME_FIELD)
        ts = frame.get_last_valid_packet_timestamp()
        frame.add_field('ZONE_PACKET_TIMESTAMP', np.array([ts], np.uint64), FieldClass.FRAME_FIELD)

        print([(zone.id, zone.trigger_status) for zone in frame.field('ZONE_STATES') if zone.live])
    # [doc-etag-zone-monitor-emulation]
