import numpy as np
from ouster.sdk.zone_monitor import ZoneSet, ZoneMode, ZoneState, Zrb
import logging
from typing import Dict, List


logger = logging.Logger(__name__)


MAX_ACTIVE_ZONES = 16
MAX_AVAILABLE_ZONES = 128


class EmulatedZoneMon:
    """Emulates ZoneMon alert decisions using ZoneRenderCache and range data."""

    def __init__(
        self, zone_set: ZoneSet
    ):
        if not zone_set.zones:
            raise ValueError("ZoneSet must have at least one zone defined")

        if not all([zone.zrb is not None for zone in zone_set.zones.values()]):
            raise ValueError("EmulatedZoneMon: all zones in ZoneSet must have a valid ZRB")

        self.zone_set = zone_set
        self.zone_counts: Dict[int, int] = {}
        self.occlusion_counts: Dict[int, int] = {}
        self.invalid_counts: Dict[int, int] = {}
        self.zone_mins: Dict[int, int] = {}
        self.zone_maxes: Dict[int, int] = {}
        self.zone_avgs: Dict[int, int] = {}
        self.zone_triggers = [0] * MAX_AVAILABLE_ZONES
        self.zone_alerts = [0] * MAX_AVAILABLE_ZONES
        self.triggered_zone_ids: List[int] = []
        self.update_count = 0
        self.rendered_zones: Dict[int, Zrb] = {
            zone_id: zone_config.zrb for zone_id, zone_config in
            zone_set.zones.items()
        }
        self.live_zones = self.zone_set.power_on_live_ids
        self.debug = False
        self.max_counts: Dict[int, int] = {
            zone_id: np.count_nonzero(zone.zrb.near_range_mm < zone.zrb.far_range_mm) for zone_id, zone in
            zone_set.zones.items()
        }

    def _calc_counts(self, range_field, bitmask_field):
        counts = {}
        occlusion_counts = {}
        invalid_counts = {}
        min_range = {}
        max_range = {}
        avg_range = {}
        for zone_id, zone in self.zone_set.zones.items():
            is_live = zone_id in self.live_zones
            if is_live:
                rendered_zone = self.rendered_zones[zone_id]
                trigger_mask = np.logical_and(range_field > 0, np.logical_and(
                    (rendered_zone.near_range_mm <= range_field), (range_field <= rendered_zone.far_range_mm)
                ))
                counts[zone_id] = np.count_nonzero(trigger_mask)
                invalid_counts[zone_id] = np.count_nonzero(np.logical_and(
                    range_field == 0, rendered_zone.near_range_mm > 0))
                occlusion_counts[zone_id] = np.count_nonzero(np.logical_and(
                    range_field > 0, range_field <= rendered_zone.near_range_mm))

                triggered_pts = range_field[trigger_mask]
                if len(triggered_pts) > 0:
                    avg_range[zone_id] = np.mean(triggered_pts)
                    min_range[zone_id] = np.min(triggered_pts)
                    max_range[zone_id] = np.max(triggered_pts)
                else:
                    avg_range[zone_id] = 0
                    min_range[zone_id] = 0
                    max_range[zone_id] = 0
                live_index = self.live_zones.index(zone_id)
                bitmask_field[trigger_mask] |= (1 << live_index)
        return counts, occlusion_counts, invalid_counts, min_range, max_range, avg_range

    def calc_triggers(self, range_field, bitmask_field = None):
        self.zone_counts, self.occlusion_counts, self.invalid_counts, self.zone_mins, self.zone_maxes, self.zone_avgs =\
             self._calc_counts(range_field, bitmask_field)
        for zone_id, zone in self.zone_set.zones.items():
            if zone_id in self.live_zones:
                if (
                        self.zone_counts[zone_id] >=
                        zone.point_count and
                        self.zone_set.zones[zone_id].mode ==
                        ZoneMode.OCCUPANCY
                ) or (
                        self.zone_counts[zone_id] <
                        zone.point_count and
                        self.zone_set.zones[zone_id].mode ==
                        ZoneMode.VACANCY
                ):
                    self.zone_triggers[zone_id] += 1
                else:
                    self.zone_triggers[zone_id] = 0
                if self.zone_triggers[zone_id] >= zone.frame_count:
                    self.zone_alerts[zone_id] += 1
                else:
                    self.zone_alerts[zone_id] = 0

        self.triggered_zone_ids = [
            zone_id for zone_id, alerts in enumerate(self.zone_alerts) if alerts > 0
        ]
        if self.debug:
            logger.info(self.zone_counts)
            logger.info(self.triggered_zone_ids)

    def get_packet(self):
        zmu = np.zeros(
            (MAX_ACTIVE_ZONES,), dtype=ZoneState.dtype()
        )
        for zone_id_idx in range(MAX_ACTIVE_ZONES):
            if zone_id_idx < len(self.live_zones):
                zone_id = self.live_zones[zone_id_idx]
                zmu[zone_id_idx]['live'] = 1
                zmu[zone_id_idx]['id'] = zone_id
                zmu[zone_id_idx]['count'] = self.zone_counts[zone_id]
                zmu[zone_id_idx]['occlusion_count'] = self.occlusion_counts[zone_id]
                zmu[zone_id_idx]['invalid_count'] = self.invalid_counts[zone_id]
                zmu[zone_id_idx]['max_count'] = self.max_counts[zone_id]
                zmu[zone_id_idx]['trigger_status'] = self.zone_alerts[zone_id] > 0
                zmu[zone_id_idx]['trigger_type'] = self.zone_set.zones[zone_id].mode.value
                zmu[zone_id_idx]['triggered_frames'] = self.zone_alerts[zone_id]
                zmu[zone_id_idx]['min_range'] = self.zone_mins[zone_id]
                zmu[zone_id_idx]['max_range'] = self.zone_maxes[zone_id]
                zmu[zone_id_idx]['mean_range'] = self.zone_avgs[zone_id]
                # OSDK-747 support error_flags
            else:
                zmu[zone_id_idx]['id'] = 255
        return zmu.view(np.recarray)
