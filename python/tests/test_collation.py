"""
Copyright (c) 2026, Ouster, Inc.
"""

import copy
import pytest
import numpy as np
from ouster.sdk import open_source, core
from typing import List, Iterator


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


# dummy source that just replays a list of frames with the given sensor infos
class FakeSrc(core.FrameSetSource):
    def __init__(self, frames, sensor_infos) -> None:
        core.FrameSetSource.__init__(self)
        self._sensor_info = sensor_infos
        self._frames = frames

        # now sort frames by start timestamp (as OSF would)
        self._frames.sort(key=lambda frame: frame.get_first_valid_packet_timestamp())

    @property
    def sensor_info(self) -> List[core.SensorInfo]:
        return self._sensor_info

    @property
    def is_live(self) -> bool:
        return False

    @property
    def is_indexed(self) -> bool:
        return False

    @property
    def frames_num(self) -> List[int]:
        return [0] * len(self._sensor_info)

    def __len__(self) -> int:
        return 0

    def __length_hint__(self) -> int:
        return 0

    def __iter__(self) -> Iterator[core.FrameSet]:
        for frame in self._frames:
            yield core.FrameSet([frame])

    def close(self) -> None:
        pass

    def __del__(self) -> None:
        self.close()


def test_collate_detect_unsynchronized(input_osf_file) -> None:
    """Validate that collation properly autodetects unsynchronized sources"""
    src = open_source(str(input_osf_file), collate=False)
    csrc = core.collate(src, dt = 0)

    # the period should be the default for a single-sensor (unsynchronized) source
    assert csrc.collation_period() == 0.21 * 1e9


def generate_test_src(frame, sensor_offsets, skip_first_frame=False,
                      synced=True, num_frames=3, imu=False, zm=False):
    num_sensors = len(sensor_offsets)
    period = 0.1
    sensor_infos = []
    frames = []
    for idx in range(num_sensors):
        si = copy.copy(frame.sensor_info)
        si.sn = idx + 10000
        si.config.timestamp_mode = core.TimestampMode.TIME_FROM_PTP_1588
        si.config.lidar_mode = core.LidarMode._1024x10
        si.format.fps = 10
        si.config.azimuth_window = (0, 360000)

        if imu:
            si.config.udp_profile_imu = core.UDPProfileIMU.ACCEL32_GYRO32_NMEA
            si.config.udp_port_imu = 100
        else:
            si.config.udp_profile_imu = core.UDPProfileIMU.LEGACY
            si.config.udp_port_imu = 0

        if zm:
            si.config.udp_port_zm = 100
            si.config.udp_dest_zm = "127.0.0.1"
        else:
            si.config.udp_port_zm = 0

        if synced:
            offset = sensor_offsets[idx] / 0.1
            si.config.phase_lock_enable = True
            si.config.phase_lock_offset = int(offset * 360000.0)
        else:
            si.config.phase_lock_enable = False

        sensor_infos.append(si)
        for frame_idx in range(num_frames):
            if skip_first_frame:
                skip_first_frame = False
                continue
            frame_copy = copy.copy(frame)
            frame_copy.sensor_info = si
            frame_copy.frame_id = 1000 + frame_idx
            frame_copy.status[:] = 1
            frame_copy.timestamp[:] = 1  # just make sure they arent zero
            start = (frame_idx * period + sensor_offsets[idx]) * 1e9
            num_pkts = frame_copy.packet_timestamp.size
            for i in range(num_pkts):
                frame_copy.packet_timestamp[i] = start + 1e9 / float(num_pkts) * i * period
            frames.append(frame_copy)
    return FakeSrc(frames, sensor_infos)


def test_collate_synchronized(input_osf_file) -> None:
    """Validate that simple synchronized collation works as expected"""
    src = open_source(str(input_osf_file), collate=False)
    frame = src[0][0]

    test_src = generate_test_src(frame, sensor_offsets=[0, 0, 0], skip_first_frame=False)

    # first make sure dt still works with a synchronized source
    assert core.collate(test_src, dt = 1).collation_period() == 1

    # then finally try autodetect, it should pick the period of the lidar
    csrc = core.collate(test_src, dt = 0)
    assert csrc.collation_period() == int(0.1 * 1e9)

    # make sure everything was collated as expected
    for idx, frame_set in enumerate(csrc):
        fid = 1000 + idx
        for sidx, frame in enumerate(frame_set):
            assert frame is not None
            assert frame.frame_id == fid
            assert frame.sensor_info.sn == 10000 + sidx

    # test that collation handles a skipped frame in the first set correctly
    test_src = generate_test_src(frame, sensor_offsets=[0, 0, 0], skip_first_frame=True)
    csrc = core.collate(test_src, dt = 0)
    assert csrc.collation_period() == int(0.1 * 1e9)

    # make sure everything was collated as expected (first frame missing from first set)
    for idx, frame_set in enumerate(csrc):
        fid = 1000 + idx
        for sidx, frame in enumerate(frame_set):
            if idx == 0 and sidx == 0:
                assert frame is None
                continue
            assert frame is not None
            assert frame.frame_id == fid
            assert frame.sensor_info.sn == 10000 + sidx


def test_collate_synchronized_offset(input_osf_file) -> None:
    """Validate that synchronized collation works as expected with phase offsets"""
    src = open_source(str(input_osf_file), collate=False)
    frame = src[0][0]

    test_src = generate_test_src(frame, sensor_offsets=[0, 0.09], skip_first_frame=False)

    # make sure autodetect picks the period of the lidar and proper latencies
    csrc = core.collate(test_src, dt=0)
    assert csrc.collation_period() == int(0.1 * 1e9)

    # check that the latency is minimized as expected (latency is in ns so use a bit of slop)
    assert np.isclose(csrc.collation_latencies(), [int(0.01 * 1e9), 0], atol=1000).all()

    # make sure everything was collated as expected
    # collation tries to minimize set latency, so frames from the second sensor
    # always come one set later (first and last set are missing opposite sensors)
    for idx, frame_set in enumerate(csrc):
        fid = 1000 + idx
        for sidx, frame in enumerate(frame_set):
            if sidx == 0:
                fid = 1000 + idx
            else:
                fid = 1000 + (idx - 1)
            if (idx == 0 and sidx == 1) or (idx == 3 and sidx == 0):
                assert frame is None
                continue
            assert frame is not None
            assert frame.frame_id == fid
            assert frame.sensor_info.sn == 10000 + sidx


@pytest.mark.parametrize('configs', [([0, 0], False, False, (0, 360000), [0, 0]),
                                     ([0.09, 0.09], False, False, (0, 360000), [0, 0]),
                                     ([0.09, 0.08], False, False, (0, 360000), [0.01, 0]),
                                     ([0.05, 0.01], False, False, (0, 180000), [0, 0.01]),
                                     ([0, 0], False, False, (0, 180000), [0.05, 0]),
                                     ([0, 0], True, False, (0, 180000), [0, 0]),
                                     ([0, 0], False, True, (0, 180000), [0, 0]),
                                     ([0, 0], True, True, (0, 180000), [0, 0]),
                                     ([0, 0.01], True, True, (0, 180000), [0, 0.01])])
def test_collate_synchronized_phases(input_osf_file, configs) -> None:
    """Validate that synchronized collation detects phase offsets correctly"""
    frame = open_source(str(input_osf_file), collate=False)[0][0]

    offset = configs[0]
    imu = configs[1]
    zm = configs[2]
    aziw = configs[3]
    expected = np.array(configs[-1]) * 1e9

    test_src = generate_test_src(frame, num_frames=0,
                                 sensor_offsets=offset, imu=imu, zm=zm)
    test_src.sensor_info[0].config.azimuth_window = aziw

    # check that it autodetects latencies as expected with a range of cases
    # make sure autodetect picks the period of the lidar and proper latencies
    csrc = core.collate(test_src, dt = 0)
    assert csrc.collation_period() == int(0.1 * 1e9)

    print(csrc.collation_latencies(), expected, aziw)
    # check that the latency is minimized as expected (latency is in ns so use a bit of slop)
    assert np.isclose(csrc.collation_latencies(), expected, atol=1000).all()
