import zipfile
import pathlib
import pytest
import json
import numpy as np
from ouster.sdk.core import SensorInfo, FieldClass
from ouster.sdk.zone_monitor import ZoneSet, Zone, ZoneMode, Stl, Zrb, \
    CoordinateFrame, EmulatedZoneMon, ZoneSetOutputFilter
from ouster.sdk import open_source


def create_test_zone_set(test_data_dir: pathlib.Path):
    sensor_info_json = open(test_data_dir / "zone_monitor" / "785.json").read()
    sensor_info = SensorInfo(sensor_info_json)
    zone_set = ZoneSet()
    sensor_to_body_transform = np.eye(4)
    sensor_to_body_transform[2, 3] = 1.0
    zone_set.sensor_to_body_transform = sensor_to_body_transform
    zone_set.power_on_live_ids = [0, 1]
    for i in range(2):
        stl_path = test_data_dir / f"zone_monitor/{i}.stl"
        zone = Zone()
        zone.point_count = 50
        zone.frame_count = 2
        zone.stl = Stl(str(stl_path))
        zone.stl.coordinate_frame = CoordinateFrame.BODY
        zone.mode = ZoneMode.OCCUPANCY
        zone_set.zones[i] = zone
    zone_set.zones[1].frame_count = 4  # make one zone different
    zone_set.render(sensor_info)
    return zone_set


def test_zone_set_zip(test_data_dir, tmp_path):
    """Reading and writing a zone set config zip should
    result in an archive with the same contents as the original."""
    zip_path = tmp_path / "test_6_zones_zmcfg.zip"
    result_zip_path = tmp_path / "result_zsc.zip"

    test_zsc = create_test_zone_set(test_data_dir)
    test_zsc.save(str(zip_path), ZoneSetOutputFilter.STL_AND_ZRB)
    with zipfile.ZipFile(zip_path, 'r') as original_zip:
        assert 'metadata.json' in original_zip.namelist()

    # importantly, we're not comparing the bytes because we
    # don't necessarily want to guarantee that the archives themselves are identical,
    # just that the contents are the same.
    zsc = ZoneSet(str(zip_path))
    zsc.save(str(result_zip_path), ZoneSetOutputFilter.STL_AND_ZRB)
    result_zsc = ZoneSet(str(result_zip_path))
    assert zsc == result_zsc


def test_zone_set_to_json_stl(test_data_dir):
    zone_set = ZoneSet()
    zone_set.sensor_to_body_transform = np.eye(4)
    zone_set.power_on_live_ids = [0]
    zone = Zone()
    zone.point_count = 1
    zone.frame_count = 1
    zone.mode = ZoneMode.OCCUPANCY
    stl = Stl(f'{test_data_dir}/zone_monitor/0.stl')
    stl.coordinate_frame = CoordinateFrame.BODY
    zone.stl = stl
    zone_set.zones = {0: zone}
    res = json.loads(zone_set.to_json(ZoneSetOutputFilter.STL))
    assert res == {
        'power_on_live_ids': zone_set.power_on_live_ids,
        'sensor_to_body_transform': zone_set.sensor_to_body_transform.flatten().tolist(),
        'version': {
            'file_naming': 1,
            'metadata': 1
        },
        'label': '',
        'zones': {
            '0': {
                'label': '',
                'frame_count': 1,
                'mode': str(zone.mode.name),
                'point_count': 1,
                'stl': {
                    'file_name': '0.stl',
                    'coordinate_frame': str(stl.coordinate_frame.name),
                    'hash': stl.hash
                }
            }
        }
    }


def test_zone_set_to_json_zrb(test_data_dir, tmp_path):
    zip_path = tmp_path / "test_6_zones_zmcfg.zip"

    test_zsc = create_test_zone_set(test_data_dir)
    test_zsc.save(str(zip_path), ZoneSetOutputFilter.STL_AND_ZRB)

    zsc = ZoneSet(str(zip_path))
    zsc_json = json.loads(zsc.to_json(ZoneSetOutputFilter.STL_AND_ZRB))
    for zone in zsc_json['zones'].values():
        del zone['zrb']['hash']  # remove hash since it is non-deterministic
    assert zsc_json == {
            "power_on_live_ids": zsc.power_on_live_ids,
            "sensor_to_body_transform": zsc.sensor_to_body_transform.flatten().tolist(),
            "version": {
                "file_naming": 1,
                "metadata": 1
            },
            'label': '',
            "zones": {
                "0": {
                    'label': '',
                    "frame_count": 2,
                    "mode": "OCCUPANCY",
                    "point_count": 50,
                    'stl': {
                        'coordinate_frame': 'BODY',
                        'file_name': '0.stl',
                        'hash': '9cb392667efd9bb1dd2f02c138049243a6103b4a0ef86574681c0641a195c7fd',
                    },
                    "zrb": {
                        "file_name": "0.zrb",
                    }
                },
                "1": {
                    'label': '',
                    "frame_count": 4,
                    "mode": "OCCUPANCY",
                    "point_count": 50,
                    'stl': {
                        'coordinate_frame': 'BODY',
                        'file_name': '1.stl',
                        'hash': '5dd053e7a8682674e9b4cdd7f48d1ffd5c3ffc6aebe20b0f4a58f7da9d210fbd',
                    },
                    "zrb": {
                        "file_name": "1.zrb",
                    }
                }
            }
    }


def test_zone_set_properties(test_data_dir):
    zsc = create_test_zone_set(test_data_dir)
    expected_sensor_to_body_transform = np.eye(4)
    expected_sensor_to_body_transform[2, 3] = 1.0
    assert np.array_equal(zsc.sensor_to_body_transform, expected_sensor_to_body_transform)
    assert set(zsc.zones.keys()) == set(range(2))
    assert zsc.power_on_live_ids == list(range(2))
    expected_zone = Zone()
    expected_zone.point_count = 50
    expected_zone.frame_count = 4
    expected_zone.mode = ZoneMode.OCCUPANCY
    zone = zsc.zones[1]
    assert zone.point_count == expected_zone.point_count
    assert zone.frame_count == expected_zone.frame_count
    assert zone.mode == expected_zone.mode


def test_mesh_bindings(test_data_dir):
    # note, these tests are somewhat redundant and only serve to verify that the python bindings work properly
    zsc = create_test_zone_set(test_data_dir)
    zone = zsc.zones[0]
    assert zone.zrb is not None
    mesh = zone.stl.to_mesh()
    assert len(mesh.triangles) > 0
    triangle = mesh.triangles[0]
    assert len(triangle.coords) == 3
    coord_a = np.array([-0.05188167, 2.3761053, 0.8726386])
    coord_b = np.array([-1.8653536, 2.1534388, 1.6861119])
    coord_c = np.array([-2.1566067, 0.50841117, 0.5865412])
    assert np.allclose(triangle.coords[0], coord_a)
    assert np.allclose(triangle.coords[1], coord_b)
    assert np.allclose(triangle.coords[2], coord_c)
    assert np.allclose(triangle.edges[0], coord_b - coord_a)
    assert np.allclose(triangle.edges[1], coord_c - coord_b)
    assert np.allclose(triangle.edges[2], coord_a - coord_c)
    assert len(triangle.edges) == 3
    normal = np.cross(triangle.edges[0], triangle.edges[1])
    normal /= np.linalg.norm(normal)
    assert np.allclose(triangle.normal, normal)


def test_zrb_from_stl(test_data_dir):
    sensor_info = SensorInfo(open(f'{test_data_dir}/zone_monitor/785.json').read())
    zone = Zone()
    zone.point_count = 1
    zone.frame_count = 1
    zone.mode = ZoneMode.OCCUPANCY
    zone.stl = Stl(f'{test_data_dir}/zone_monitor/0.stl')
    zone.stl.coordinate_frame = CoordinateFrame.BODY
    assert zone.stl.coordinate_frame == CoordinateFrame.BODY
    zone_set = ZoneSet()
    zone_set.sensor_to_body_transform = np.eye(4)
    # test assignment from dict
    zone_set.zones = {0: zone}
    assert zone_set.zones[0] == zone
    zone_set.render(sensor_info)
    zrb = zone_set.zones[0].zrb
    assert zrb.stl_hash == zone.stl.hash
    assert zrb.near_range_mm.shape == (sensor_info.h, sensor_info.w)
    assert zrb.serial_number == sensor_info.sn
    assert zrb is not None


def test_zone_render_older_fw(test_data_dir):
    sensor_info = SensorInfo(open(f'{test_data_dir}/pcaps/OS-0-128_v3.0.1_1024x10.2.json').read())
    assert sensor_info.fw_rev == "v3.0.1"
    zone = Zone()
    zone.point_count = 1
    zone.frame_count = 1
    zone.mode = ZoneMode.OCCUPANCY
    zone.stl = Stl(f'{test_data_dir}/zone_monitor/0.stl')
    zone.stl.coordinate_frame = CoordinateFrame.SENSOR
    zone_set = ZoneSet()
    sensor_to_body_transform = np.eye(4)
    sensor_to_body_transform[2, 3] = 1.0  # just to make it nontrivial
    zone_set.sensor_to_body_transform = sensor_to_body_transform
    zone_set.zones = {0: zone}
    zone_set.render(sensor_info)
    zrb = zone_set.zones[0].zrb
    assert zrb is not None
    assert zrb.stl_hash == zone.stl.hash
    assert zrb.near_range_mm.shape == (sensor_info.h, sensor_info.w)
    assert zrb.serial_number == sensor_info.sn


def test_emulated_zone_mon_init(test_data_dir):
    zsc = create_test_zone_set(test_data_dir)
    from ouster.sdk.zone_monitor import EmulatedZoneMon
    ezm = EmulatedZoneMon(zsc)
    assert ezm.zone_set == zsc
    assert ezm.zone_counts == {}
    assert ezm.occlusion_counts == {}
    assert ezm.invalid_counts == {}
    assert ezm.max_counts == {0: 12096, 1: 3098}
    assert ezm.zone_mins == {}
    assert ezm.zone_maxes == {}
    assert ezm.zone_avgs == {}
    assert ezm.zone_triggers == [0] * 128
    assert ezm.zone_alerts == [0] * 128
    assert ezm.triggered_zone_ids == []
    assert ezm.update_count == 0
    assert set(ezm.rendered_zones.keys()) == set(range(2))
    assert ezm.live_zones == list(range(2))
    assert not ezm.debug


def test_max_count(test_data_dir):
    zone_set = ZoneSet()
    zone_set.sensor_to_body_transform = np.eye(4)
    zone = Zone()
    zone.point_count = 1
    zone.frame_count = 1
    zone.mode = ZoneMode.OCCUPANCY
    zone.zrb = Zrb()
    zone.zrb.near_range_mm = np.ones((4, 4), dtype=np.uint32)
    zone.zrb.far_range_mm = np.ones((4, 4), dtype=np.uint32) * 5
    zone_set.zones = {0: zone}
    ezm = EmulatedZoneMon(zone_set)
    assert ezm.max_counts[0] == 16  # 4x4 grid with all near < far
    zone.zrb.near_range_mm = np.ones((4, 4), dtype=np.uint32) * 5
    zone.zrb.far_range_mm = np.ones((4, 4), dtype=np.uint32)
    zone_set.zones = {0: zone}
    ezm = EmulatedZoneMon(zone_set)
    assert ezm.max_counts[0] == 0  # 4x4 grid with all near >= far


def test_emulated_zone_mon_every_zone_must_have_a_zrb(test_data_dir):
    """It should require all zones to have a Zrb."""
    sensor_info = SensorInfo(open(f'{test_data_dir}/pcaps/OS-0-128_v3.0.1_1024x10.2.json').read())
    zone_set = create_test_zone_set(test_data_dir)
    zone_set.zones[0].zrb = None
    with pytest.raises(ValueError, match="EmulatedZoneMon: all zones in ZoneSet must have a valid ZRB"):
        EmulatedZoneMon(zone_set)
    zone_set.render(sensor_info)
    assert zone_set.zones[0].zrb is not None
    EmulatedZoneMon(zone_set)


def test_emulated_zone_mon_requires_rendered_zones(test_data_dir):
    """It should require rendered zones to be present."""
    zone_set = ZoneSet()
    zone_set.sensor_to_body_transform = np.eye(4)
    zone = Zone()
    zone.point_count = 1
    zone.frame_count = 1
    zone.mode = ZoneMode.OCCUPANCY
    zone.stl = Stl(f'{test_data_dir}/zone_monitor/0.stl')
    zone.stl.coordinate_frame = CoordinateFrame.BODY
    zone_set.zones = {0: zone}
    assert zone_set.zones[0].zrb is None
    with pytest.raises(ValueError, match="EmulatedZoneMon: all zones in ZoneSet must have a valid ZRB"):
        EmulatedZoneMon(zone_set)


def test_emulated_zone_mon_new_fields(test_data_dir):
    source = open_source(f'{test_data_dir}/zone_monitor/single_frame_zm.osf')
    scan, = next(iter(source))
    zone_states = scan.field('ZONE_STATES')
    zone_zero = zone_states[0]
    assert zone_zero['id'] == 0
    assert zone_zero['live'] == 1
    assert zone_zero['error_flags'] == 0
    assert zone_zero['min_range'] == 674
    assert zone_zero['max_range'] == 1019
    assert zone_zero['mean_range'] == 832
    assert zone_zero['count'] == 1094
    assert zone_zero['trigger_type'] == ZoneMode.OCCUPANCY.value
    assert zone_zero['trigger_status'] == 1
    assert zone_zero['triggered_frames'] == 10553
    assert zone_zero['occlusion_count'] == 837
    assert zone_zero['invalid_count'] == 1093
    assert zone_zero['max_count'] == 3439

    sensor_info = source.sensor_info[0]
    ezm = EmulatedZoneMon(sensor_info.zone_set)
    bitmask_field = np.zeros((sensor_info.h, sensor_info.w), dtype=np.uint32)
    ezm.calc_triggers(scan.field('RANGE'), bitmask_field)
    ezm.calc_triggers(scan.field('RANGE'), bitmask_field)  # to increment triggered_frames and cause a trigger
    zone_states_2 = ezm.get_packet()
    zone_zero_2 = zone_states_2[0]
    assert zone_zero_2['id'] == zone_zero['id']
    assert zone_zero_2['live'] == zone_zero['live']
    assert zone_zero_2['error_flags'] == zone_zero['error_flags']

    # IMPORTANT - these values will differ because we're only emulating two frames
    # versus the original data which was recorded over many frames
    assert zone_zero_2['min_range'] == 676
    assert zone_zero_2['max_range'] == 1020
    assert zone_zero_2['mean_range'] == 836
    assert zone_zero_2['count'] == 1089
    assert zone_zero_2['trigger_type'] == zone_zero['trigger_type']
    assert zone_zero_2['trigger_status'] == 1
    # NOTE - only 1 because we're only emulating two frames
    # but the dataset was recorded on a sensor that was running for a long time
    assert zone_zero_2['triggered_frames'] == 1
    assert zone_zero_2['occlusion_count'] == 847  # somewhat different because near/far ranges differ slightly
    assert zone_zero_2['invalid_count'] == 1140
    assert zone_zero_2['max_count'] == zone_zero['max_count']


def test_emulated_zone_mon_vacancy_mode(test_data_dir):
    # The single frame we used to test OCCUPANCY above should not trigger in VACANCY mode
    source = open_source(f'{test_data_dir}/zone_monitor/single_frame_zm.osf')
    sensor_info = source.sensor_info[0]
    sensor_info.zone_set.zones[0].mode = ZoneMode.VACANCY
    ezm = EmulatedZoneMon(sensor_info.zone_set)
    scan, = next(iter(source))
    bitmask_field = np.zeros((sensor_info.h, sensor_info.w), dtype=np.uint32)
    ezm.calc_triggers(scan.field('RANGE'), bitmask_field)
    ezm.calc_triggers(scan.field('RANGE'), bitmask_field)  # to increment triggered_frames and cause a trigger
    zone_states_2 = ezm.get_packet()
    zone_zero_2 = zone_states_2[0]
    assert zone_zero_2['trigger_type'] == ZoneMode.VACANCY.value
    assert zone_zero_2['trigger_status'] == 0
    assert zone_zero_2['triggered_frames'] == 0


def test_emulated_zone_mon_vacancy_mode_2(test_data_dir):
    # When all points are further than the zone, then VACANCY should trigger
    source = open_source(f'{test_data_dir}/zone_monitor/single_frame_zm.osf')
    sensor_info = source.sensor_info[0]
    sensor_info.zone_set.zones[0].mode = ZoneMode.VACANCY
    ezm = EmulatedZoneMon(sensor_info.zone_set)
    scan, = next(iter(source))
    bitmask_field = np.zeros((sensor_info.h, sensor_info.w), dtype=np.uint32)
    ezm.calc_triggers(scan.field('RANGE') + 1000, bitmask_field)
    ezm.calc_triggers(scan.field('RANGE') + 1000, bitmask_field)  # to increment triggered_frames and cause a trigger
    zone_states_2 = ezm.get_packet()
    zone_zero_2 = zone_states_2[0]
    assert zone_zero_2['trigger_type'] == ZoneMode.VACANCY.value
    assert zone_zero_2['trigger_status'] == 1
    assert zone_zero_2['triggered_frames'] == 1


def test_blob_renders_zrb_zones(test_data_dir):
    """It should not render zones for an STL ZoneSet."""
    zone_set = ZoneSet()
    zone_set.sensor_to_body_transform = np.eye(4)
    zone = Zone()
    zone.point_count = 1
    zone.frame_count = 1
    zone.mode = ZoneMode.OCCUPANCY
    zone.stl = Stl(f'{test_data_dir}/zone_monitor/0.stl')
    zone.stl.coordinate_frame = CoordinateFrame.BODY
    zone_set.zones = {0: zone}
    zone_set_bytes = zone_set.to_zip_blob(ZoneSetOutputFilter.STL_AND_ZRB)
    zone_set_2 = ZoneSet(zone_set_bytes)
    assert zone_set_2.zones[0].zrb is None


def test_emulated_zone_mon_get_packet(test_data_dir):
    """It should calculate counts correctly."""
    sensor_info = SensorInfo(open(f'{test_data_dir}/pcaps/OS-0-128_v3.0.1_1024x10.2.json').read())
    zone_set = ZoneSet()
    zone_set.sensor_to_body_transform = np.eye(4)
    zone = Zone()
    zone.point_count = 1
    zone.frame_count = 1
    zone.mode = ZoneMode.OCCUPANCY
    zone.stl = Stl(f'{test_data_dir}/zone_monitor/0.stl')
    zone.stl.coordinate_frame = CoordinateFrame.BODY
    zone_set.zones = {0: zone}
    zone_set.render(sensor_info)
    zone_set.power_on_live_ids = [0]
    ezm = EmulatedZoneMon(zone_set)

    # create a fake range field with points in the zone
    fixed_range = 1000  # mm
    range_field = np.full((sensor_info.h, sensor_info.w), fixed_range, dtype=np.uint32)
    bitmask_field = np.zeros((sensor_info.h, sensor_info.w), dtype=np.uint32)
    ezm.calc_triggers(range_field, bitmask_field)
    packet = ezm.get_packet()
    assert packet[0]['id'] == 0
    assert packet[0]['live'] == 1
    assert packet[0]['count'] == 1218
    assert packet[0]['min_range'] == fixed_range
    assert packet[0]['max_range'] == fixed_range
    assert packet[0]['mean_range'] == fixed_range
    assert packet[0]['trigger_status'] == 1
    assert packet[0]['triggered_frames'] == 1


def test_point_count_sanity_check(test_data_dir):
    """It should fail if a zrb has no data."""
    sensor_info = SensorInfo(open(f'{test_data_dir}/pcaps/OS-0-128_v3.0.1_1024x10.2.json').read())
    zone_set = ZoneSet()
    zone_set.sensor_to_body_transform = np.eye(4)
    zone1 = Zone()
    zone1.point_count = 1
    zone1.frame_count = 1
    zone1.mode = ZoneMode.OCCUPANCY
    zone1.zrb = Zrb()
    zone1.zrb.serial_number = sensor_info.sn
    zone1.zrb.near_range_mm = np.zeros((sensor_info.h, sensor_info.w), dtype=np.uint32)
    zone1.zrb.far_range_mm = np.zeros((sensor_info.h, sensor_info.w), dtype=np.uint32)
    zone_set.zones = {1: zone1}
    with pytest.raises(RuntimeError, match="ZoneSet: Zone 1 failed invariant check: Zone: ZRB "
        "far range image has fewer nonzero pixels than point_count"):
        zone_set.to_json(ZoneSetOutputFilter.STL_AND_ZRB)


def test_fail_no_stl_and_no_zrb(test_data_dir):
    """It should require each zone to have either an STL or a ZRB."""
    zone_set = ZoneSet()
    zone_set.sensor_to_body_transform = np.eye(4)
    zone = Zone()
    zone.point_count = 1
    zone.frame_count = 1
    zone.mode = ZoneMode.OCCUPANCY
    zone_set.zones = {0: zone}
    with pytest.raises(RuntimeError, match="ZoneSet: Zone 0 failed invariant check: Zone: must have either STL or ZRB"):
        zone_set.to_zip_blob(ZoneSetOutputFilter.STL_AND_ZRB)


def test_it_should_fail_with_invalid_zone_id(test_data_dir):
    """It should require zone_id to be a valid value."""
    zone_set = ZoneSet()
    zone_set.sensor_to_body_transform = np.eye(4)
    zone = Zone()
    zone.point_count = 1
    zone.frame_count = 1
    zone.mode = ZoneMode.OCCUPANCY
    zone.stl = Stl(f'{test_data_dir}/zone_monitor/0.stl')
    zone.stl.coordinate_frame = CoordinateFrame.BODY
    with pytest.raises(RuntimeError, match="Additional property \"128\" found but was invalid."):
        zone_set.zones = {128: zone}
        zone_set.to_zip_blob(ZoneSetOutputFilter.STL_AND_ZRB)


def test_it_should_fail_with_invalid_power_on_live_ids(test_data_dir):
    """It should require power_on_live_ids to be valid values."""
    zone_set = ZoneSet()
    zone_set.sensor_to_body_transform = np.eye(4)
    zone = Zone()
    zone.point_count = 1
    zone.frame_count = 1
    zone.mode = ZoneMode.OCCUPANCY
    zone.stl = Stl(f'{test_data_dir}/zone_monitor/0.stl')
    zone.stl.coordinate_frame = CoordinateFrame.BODY
    zone_set.zones = {0: zone}
    with pytest.raises(RuntimeError, match="128 exceeds maximum of 127"):
        zone_set.power_on_live_ids = [128]
        zone_set.to_zip_blob(ZoneSetOutputFilter.STL_AND_ZRB)


def test_zone_set_eq(test_data_dir):
    zm1 = ZoneSet()
    zm2 = ZoneSet()
    assert zm1 == zm2
    # Just a note - comparing to None currently
    # raises TypeError for all of our __eq__ implementations
    with pytest.raises(TypeError):
        assert zm1 != None  # noqa: E711
    # But this is fine
    assert zm1 is not None


def test_zrb_zone_set_save_without_render(test_data_dir):
    zone_set = ZoneSet()
    zone_set.sensor_to_body_transform = np.eye(4)
    zone_0 = Zone()
    zone_0.point_count = 1
    zone_0.frame_count = 1
    zone_0.mode = ZoneMode.OCCUPANCY
    zone_0.stl = Stl(f'{test_data_dir}/zone_monitor/0.stl')
    zone_0.stl.coordinate_frame = CoordinateFrame.BODY
    zone_set.zones = {
        0: zone_0
    }

    # no ZRBs because we didn't render
    assert not any(
        zone.zrb
        for zone in ZoneSet(
            zone_set.to_zip_blob(
                ZoneSetOutputFilter.STL_AND_ZRB)
        ).zones.values()
    )

    # rendering should populate ZRBs
    zone_set.render(SensorInfo(open(f'{test_data_dir}/pcaps/OS-0-128_v3.0.1_1024x10.2.json').read()))
    assert all(zone.zrb for zone in zone_set.zones.values())


def test_saving_preserves_stl_filename(test_data_dir):
    zone_set = ZoneSet()
    zone_set.sensor_to_body_transform = np.eye(4)
    zone = Zone()
    zone.point_count = 1
    zone.frame_count = 1
    zone.mode = ZoneMode.OCCUPANCY
    zone.stl = Stl(f'{test_data_dir}/zone_monitor/ascii.stl')
    zone.stl.coordinate_frame = CoordinateFrame.BODY
    zone_set.zones = {0: zone}
    metadata = json.loads(zone_set.to_json(ZoneSetOutputFilter.STL))
    assert metadata['zones']['0']['stl'] == {
        'file_name': 'ascii.stl',
        'coordinate_frame': 'BODY',
        'hash': zone.stl.hash
    }
    zone_set_2 = ZoneSet(zone_set.to_zip_blob(ZoneSetOutputFilter.STL))
    assert zone_set_2.zones[0].stl.filename == 'ascii.stl'


def test_labels(test_data_dir):
    zone_set = ZoneSet()
    zone_set.label = 'abc'
    zone_set.sensor_to_body_transform = np.eye(4)
    zone = Zone()
    zone.point_count = 1
    zone.frame_count = 1
    zone.mode = ZoneMode.OCCUPANCY
    zone.label = 'def'
    stl = Stl(f'{test_data_dir}/zone_monitor/0.stl')
    stl.coordinate_frame = CoordinateFrame.BODY
    zone.stl = stl
    zone_set.zones = {0: zone}
    zone_set_json = json.loads(zone_set.to_json(ZoneSetOutputFilter.STL))
    assert zone_set_json['label'] == zone_set.label


def test_filename_case(test_data_dir, tmp_path):
    """It should handle an STL filename with a different case from what's provided in the metadata."""
    zip_filename = tmp_path / "test_case_zip.zip"
    with zipfile.ZipFile(zip_filename, 'w') as zf:
        zf.write(test_data_dir / "zone_monitor/0.stl", "0.STL")
        metadata = {
            "power_on_live_ids": [0],
            "sensor_to_body_transform": np.eye(4).flatten().tolist(),
            "version": {
                "file_naming": 1,
                "metadata": 1
            },
            "label": "",
            "zones": {
                "0": {
                    "label": "",
                    "frame_count": 1,
                    "mode": "OCCUPANCY",
                    "point_count": 1,
                    "stl": {
                        "file_name": "0.stl",
                        "coordinate_frame": "BODY",
                        "hash": Stl(str(test_data_dir / "zone_monitor/0.stl")).hash
                    }
                }
            }
        }
        zf.writestr("metadata.json", json.dumps(metadata))
    # It doesn't throw
    ZoneSet(str(zip_filename))


def test_zone_states_dtype(test_data_dir):
    """It should have the correct dtype for zone states."""
    source = open_source(f'{test_data_dir}/zone_monitor/single_frame_zm.osf')
    scan, = next(iter(source))
    expected_dtype = np.dtype((np.record, [
            ('live', 'u1'),
            ('id', 'u1'),
            ('error_flags', 'u1'),
            ('trigger_type', 'u1'),
            ('trigger_status', 'u1'),
            ('triggered_frames', '<u4'),
            ('count', '<u4'),
            ('occlusion_count', '<u4'),
            ('invalid_count', '<u4'),
            ('max_count', '<u4'),
            ('min_range', '<u4'),
            ('max_range', '<u4'),
            ('mean_range', '<u4')
    ]))
    zone_states = scan.field('ZONE_STATES')
    assert zone_states.dtype == expected_dtype
    zone_states[0].live

    emulated_zm = EmulatedZoneMon(source.sensor_info[0].zone_set)
    emulated_zm.calc_triggers(
        scan.field('RANGE'),
        np.zeros((source.sensor_info[0].h, source.sensor_info[0].w), dtype=np.uint32)
    )
    zone_states_emulated = emulated_zm.get_packet()
    assert zone_states_emulated.dtype == expected_dtype
    zone_states_emulated[0].live

    #  add it to the scan, verify type is preserved
    scan.del_field('ZONE_STATES')
    scan.add_field('ZONE_STATES', zone_states_emulated, FieldClass.SCAN_FIELD)
    assert zone_states_emulated.dtype == expected_dtype
    zone_states_emulated[0].live
    assert scan.field('ZONE_STATES').dtype == expected_dtype
