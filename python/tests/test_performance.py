from ouster.sdk.core import dewarp, transform, interp_pose
from ouster.sdk.util.parsing import frame_to_packets  # type: ignore
from ouster.sdk import core, open_source
from ouster.sdk.algorithm import normals
import pytest
import time
import numpy as np
import copy
from click.testing import CliRunner

from ouster.cli import core as cli_core
from ouster.cli.plugins import source, source_osf  # noqa: F401

from tests.conftest import OSFS_DATA_DIR


@pytest.fixture
def runner():
    return CliRunner()


class ProfileRunner:
    def __init__(self, long_mode, record_property, iters):
        self._record_property = record_property
        self._long = long_mode
        self._start = None
        self._iters = int(iters) if iters else None

    def start(self):
        self._start = (time.time(), time.process_time())

    @property
    def long(self):
        return self._long

    def iterations(self, test_iters):
        if self._iters:
            return self._iters
        if self._long:
            return test_iters
        return 2

    def end(self, num_iters):
        end = (time.time(), time.process_time())
        dt = end[0] - self._start[0]
        per_iter = dt / num_iters
        print("Total:          ", end[0] - self._start[0], 's')
        print("Per Iteration:  ", per_iter / 0.001, 'ms')
        print("Num Iteration:  ", num_iters)
        self._record_property("test_runtime", dt)
        self._record_property("test_iteration_time", per_iter)
        self._record_property("test_iterations", num_iters)


@pytest.fixture
def profile(request, record_property):
    return ProfileRunner(request.config.getoption("--performance"), record_property,
                         request.config.getoption("--num-iterations"))


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.performance
def test_perf_destagger(frame: core.LidarFrame, meta: core.SensorInfo, profile) -> None:
    # do setup
    num_iters = profile.iterations(8000)
    rng = frame.field(core.ChanField.RANGE)
    rngs = []
    for i in range(min(num_iters, 8000)):  # limit memory usage
        cpy = np.copy(rng)
        rngs.append(cpy)

    # perform the actual test
    profile.start()
    for i in range(num_iters):
        core.destagger(meta, rngs[i % len(rngs)])
    profile.end(num_iters)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.performance
def test_perf_xyz(frame: core.LidarFrame, meta: core.SensorInfo, profile) -> None:
    # do setup
    num_iters = profile.iterations(8000)
    xyzlut = core.XYZLut(meta)
    rng = frame.field(core.ChanField.RANGE)
    rngs = []
    for i in range(min(num_iters, 8000)):  # limit memory usage
        cpy = np.copy(rng)
        rngs.append(cpy)

    # perform the actual test
    profile.start()
    for i in range(num_iters):
        xyzlut(rngs[i % len(rngs)])
    profile.end(num_iters)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.performance
def test_perf_dwarp(frame: core.LidarFrame, meta: core.SensorInfo, profile) -> None:
    # do setup
    num_iters = profile.iterations(8000)
    xyzlut = core.XYZLut(meta)
    rng = frame.field(core.ChanField.RANGE)
    xyz = xyzlut(rng)

    # perform the actual test
    profile.start()
    for i in range(num_iters):
        dewarp(xyz, frame.body_to_world)
    profile.end(num_iters)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.performance
def test_perf_transform(frame: core.LidarFrame, meta: core.SensorInfo, profile) -> None:
    # do setup
    num_iters = profile.iterations(8000)
    xyzlut = core.XYZLut(meta)
    rng = frame.field(core.ChanField.RANGE)
    xyz = xyzlut(rng)
    pose = frame.body_to_world[0]

    # perform the actual test
    profile.start()
    for i in range(num_iters):
        transform(xyz, pose)
    profile.end(num_iters)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.parametrize('happy_packet', [True, False])
@pytest.mark.performance
def test_perf_batching(frame: core.LidarFrame, meta: core.SensorInfo, profile, happy_packet) -> None:
    # do setup for batching num_iters frames worth of packets
    num_iters = profile.iterations(400)
    packets = []
    for i in range(num_iters):
        frame.frame_id = frame.frame_id + 1
        frame.status[:] = 1
        if not happy_packet:
            for i in range(64):
                frame.status[16 * i] = 0
        for packet in frame_to_packets(frame, meta):
            packets.append(packet)
    batcher = core.FrameBatcher(meta)
    new_frames = [core.LidarFrame(meta)] * num_iters

    # perform the actual test
    num_batched = 0
    profile.start()
    for p in packets:
        if batcher.batch(p, new_frames[num_batched]):
            num_batched = num_batched + 1
    profile.end(num_iters)
    assert num_batched == num_iters


def _prepare_destaggered_returns():
    """Return destaggered XYZ/range arrays and sensor origins for first/second returns."""
    src = open_source(OSFS_DATA_DIR + "/single_scan_016.osf")
    frames = next(iter(src))
    frame = frames[0]
    info = src.sensor_info[0]
    xyzlut = core.XYZLut(info)
    h, w = info.h, info.w

    def destagger_return(field_name):
        if not frame.has_field(field_name):
            return None, None
        field = frame.field(field_name)
        xyz = xyzlut(field).reshape(h, w, 3)
        xyz_destaggered = core.destagger(info, xyz)
        range_destaggered = core.destagger(info, field)
        return xyz_destaggered, range_destaggered

    first_xyz, first_range = destagger_return(core.ChanField.RANGE)
    second_xyz, second_range = destagger_return(core.ChanField.RANGE2)
    sensor_origins_xyz = np.zeros((w, 3))
    return info, sensor_origins_xyz, first_xyz, first_range, second_xyz, second_range


@pytest.mark.performance
def test_perf_normals_single_return(profile) -> None:
    info, sensor_origins_xyz, first_xyz, first_range, _, _ = _prepare_destaggered_returns()
    num_iters = profile.iterations(800)

    profile.start()
    for _ in range(num_iters):
        _ = normals(
            first_xyz, first_range,
            sensor_origins_xyz=sensor_origins_xyz)
    profile.end(num_iters)


@pytest.mark.performance
def test_perf_normals_dual_return(profile) -> None:
    info, sensor_origins_xyz, first_xyz, first_range, second_xyz, second_range = (
        _prepare_destaggered_returns())

    num_iters = profile.iterations(800)
    profile.start()
    for _ in range(num_iters):
        _ = normals(
            first_xyz,
            first_range,
            second_xyz,
            second_range,
            sensor_origins_xyz=sensor_origins_xyz,
        )
    profile.end(num_iters)


# now for each frame source type
@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.performance
def test_perf_osf_save(frame: core.LidarFrame, meta: core.SensorInfo, profile, tmp_path) -> None:
    # setup the test
    # save N frames to OSF and close
    from ouster.sdk.osf import Writer
    file_name = str(tmp_path / "test.osf")
    num_iters = profile.iterations(40)
    frames = []
    for i in range(num_iters):
        frames.append(copy.copy(frame))

    # perform the actual test
    profile.start()
    writer = Writer(file_name, meta)
    for frame in frames:
        writer.save(0, frame)
    writer.close()
    profile.end(num_iters)


@pytest.fixture(scope="session")
def tmp_osf(tmp_path_factory, request):
    # Create a fake OSF for profiling tests
    from ouster.sdk.osf import Writer
    from ouster.sdk import open_source
    fn = str(tmp_path_factory.mktemp("data") / "test.osf")
    src = open_source(OSFS_DATA_DIR + "/OS-1-128_v2.3.0_1024x10_lb_n3.osf")
    meta = src.sensor_info
    frame = next(iter(src))[0]
    writer = Writer(fn, meta)
    count = 100 if request.config.getoption("--performance") else 2
    if request.config.getoption("--num-iterations"):
        count = int(request.config.getoption("--num-iterations"))
    for i in range(count):
        frame.frame_id = frame.frame_id + 1
        frame.packet_timestamp[:] = frame.packet_timestamp + 100000000
        frame.timestamp[:] = frame.timestamp + 100000000
        writer.save(0, frame)
    writer.close()
    return fn


@pytest.fixture(scope="session")
def tmp_pcap(tmp_path_factory, request):
    # Create a fake PCAP for profiling tests
    from ouster.sdk import open_source
    import ouster.sdk._bindings.pcap as _pcap
    directory = tmp_path_factory.mktemp("data")
    fn = str(directory / "test.pcap")
    fn_json = str(directory / "test.json")
    src = open_source(OSFS_DATA_DIR + "/OS-1-128_v2.3.0_1024x10_lb_n3.osf")
    meta = src.sensor_info[0]
    frame = next(iter(src))[0]
    count = 100 if request.config.getoption("--performance") else 2
    if request.config.getoption("--num-iterations"):
        count = int(request.config.getoption("--num-iterations"))
    with open(fn_json, "w") as f:
        f.write(meta.to_json_string())

    handle = _pcap.record_initialize(fn, 2**16)
    for i in range(count):
        # convert to packets and save
        for packet in frame_to_packets(frame, meta):
            _pcap.record_packet(handle, "127.0.0.1", "127.0.0.1", meta.config.udp_port_lidar,
                                meta.config.udp_port_lidar, packet.buf, packet.host_timestamp / 1e9)

        frame.frame_id = frame.frame_id + 1
        frame.packet_timestamp[:] = frame.packet_timestamp + 100000000
        frame.timestamp[:] = frame.timestamp + 100000000

    _pcap.record_uninitialize(handle)
    return fn


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.performance
def test_perf_osf_read(frame: core.LidarFrame, meta: core.SensorInfo, profile, tmp_osf) -> None:
    # Read all frames in the OSF
    from ouster.sdk import open_source
    num_iters = 0

    profile.start()
    for i in open_source(tmp_osf):
        num_iters = num_iters + 1
    profile.end(num_iters)


@pytest.mark.performance
def test_perf_osf_open(profile, tmp_osf) -> None:
    # open the osf and read one frame N times
    from ouster.sdk import open_source
    num_iters = profile.iterations(40)

    profile.start()
    for i in range(num_iters):
        for frame in open_source(tmp_osf):
            break
    profile.end(num_iters)


@pytest.mark.performance
def test_perf_pcap_read(profile, tmp_pcap) -> None:
    # Read all frames in the PCAP
    from ouster.sdk import open_source
    num_iters = 0

    profile.start()
    for i in open_source(tmp_pcap):
        num_iters = num_iters + 1
    profile.end(num_iters)


@pytest.mark.performance
def test_perf_pcap_open(profile, tmp_pcap) -> None:
    # open the osf and read one frame N times
    from ouster.sdk import open_source
    num_iters = profile.iterations(40)

    profile.start()
    for i in range(num_iters):
        for frame in open_source(tmp_pcap):
            break
    profile.end(num_iters)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.performance
def test_perf_pcap_save(frame: core.LidarFrame, meta: core.SensorInfo, profile, tmp_path) -> None:
    # setup the test
    # save N frames to PCAP and close
    num_iters = profile.iterations(40)

    import ouster.sdk._bindings.pcap as _pcap
    fn = str(tmp_path / "test.pcap")
    fn_json = str(tmp_path / "test.json")

    # make the frames
    frames = []
    for i in range(num_iters):
        frame2 = copy.copy(frame)
        frames.append(frame2)
        frame.frame_id = frame.frame_id + 1
        frame.packet_timestamp[:] = frame.packet_timestamp + 100000000
        frame.timestamp[:] = frame.timestamp + 100000000

    # perform the actual test
    profile.start()
    with open(fn_json, "w") as f:
        f.write(meta.to_json_string())

    handle = _pcap.record_initialize(fn, 2**16)
    for frame in frames:
        # convert to packets and save
        for packet in frame_to_packets(frame, meta):
            _pcap.record_packet(handle, "127.0.0.1", "127.0.0.1", meta.config.udp_port_lidar or 0,
                                meta.config.udp_port_lidar or 0, packet.buf, packet.host_timestamp / 1e9)
    _pcap.record_uninitialize(handle)
    profile.end(num_iters)


@pytest.mark.performance
def test_perf_pcap_cli_read(profile, tmp_pcap, runner) -> None:
    # open the file and run stats on it
    num_iters = 1

    profile.start()
    result = runner.invoke(cli_core.cli, ['source', tmp_pcap, 'stats'])  # type: ignore
    profile.end(num_iters)
    print(result.output)
    assert result.exit_code == 0


@pytest.mark.performance
def test_perf_osf_cli_read(profile, tmp_osf, runner) -> None:
    # open the file and run stats on it
    num_iters = 1

    profile.start()
    result = runner.invoke(cli_core.cli, ['source', tmp_osf, 'stats'])  # type: ignore
    profile.end(num_iters)
    print(result.output)
    assert result.exit_code == 0


@pytest.mark.performance
def test_perf_osf_cli_slice(profile, tmp_osf, runner) -> None:
    # slice the file to the last frame and run stats on it
    from ouster.sdk import open_source
    l = len(open_source(tmp_osf))

    num_iters = 1
    slice = str(l - 1) + ":"
    profile.start()
    result = runner.invoke(cli_core.cli, ['source', tmp_osf, 'slice', slice, 'stats'])  # type: ignore
    profile.end(num_iters)
    print(result.output)
    assert result.exit_code == 0


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.performance
def test_perf_interp_pose(frame: core.LidarFrame, meta: core.SensorInfo, profile) -> None:
    # do setup
    num_iters = profile.iterations(10)
    num_interp = 4096
    x_interp = np.linspace(0, 1, num_interp)

    num_known = 64
    x_known = np.linspace(0, 1, num_known)
    x_poses_list = []

    for i in range(num_known):
        tr = np.eye(4)
        # vary the position a bit
        tr[3, 0:3] = np.array([i * 0.1, i * 0.05, i * 0.02])
        x_poses_list.append(tr)
    x_poses = np.array(x_poses_list)

    # perform the actual test
    profile.start()
    for i in range(num_iters):
        interp_pose(x_interp, x_known, x_poses)
    profile.end(num_iters)
