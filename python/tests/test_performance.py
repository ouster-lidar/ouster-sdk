from ouster.sdk.core import dewarp, transform
from ouster.sdk.util.parsing import scan_to_packets  # type: ignore
from ouster.sdk import core
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
def test_perf_destagger(scan: core.LidarScan, meta: core.SensorInfo, profile) -> None:
    # do setup
    num_iters = profile.iterations(8000)
    rng = scan.field(core.ChanField.RANGE)
    rngs = []
    for i in range(num_iters):
        cpy = np.copy(rng)
        rngs.append(cpy)

    # perform the actual test
    profile.start()
    for i in range(num_iters):
        core.destagger(meta, rngs[i])
    profile.end(num_iters)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.performance
def test_perf_xyz(scan: core.LidarScan, meta: core.SensorInfo, profile) -> None:
    # do setup
    num_iters = profile.iterations(8000)
    xyzlut = core.XYZLut(meta)
    rng = scan.field(core.ChanField.RANGE)
    rngs = []
    for i in range(num_iters):
        cpy = np.copy(rng)
        rngs.append(cpy)

    # perform the actual test
    profile.start()
    for i in range(num_iters):
        xyzlut(rngs[i])
    profile.end(num_iters)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.performance
def test_perf_dwarp(scan: core.LidarScan, meta: core.SensorInfo, profile) -> None:
    # do setup
    num_iters = profile.iterations(8000)
    xyzlut = core.XYZLut(meta)
    rng = scan.field(core.ChanField.RANGE)
    xyz = xyzlut(rng)

    # perform the actual test
    profile.start()
    for i in range(num_iters):
        dewarp(xyz, scan.pose)
    profile.end(num_iters)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.performance
def test_perf_transform(scan: core.LidarScan, meta: core.SensorInfo, profile) -> None:
    # do setup
    num_iters = profile.iterations(8000)
    xyzlut = core.XYZLut(meta)
    rng = scan.field(core.ChanField.RANGE)
    xyz = xyzlut(rng)
    pose = scan.pose[0]

    # perform the actual test
    profile.start()
    for i in range(num_iters):
        transform(xyz, pose)
    profile.end(num_iters)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.parametrize('happy_packet', [True, False])
@pytest.mark.performance
def test_perf_batching(scan: core.LidarScan, meta: core.SensorInfo, profile, happy_packet) -> None:
    # do setup for batching num_iters scans worth of packets
    num_iters = profile.iterations(400)
    packets = []
    for i in range(num_iters):
        scan.frame_id = scan.frame_id + 1
        scan.status[:] = 1
        if not happy_packet:
            for i in range(64):
                scan.status[16 * i] = 0
        for packet in scan_to_packets(scan, meta):
            packets.append(packet)
    batcher = core.ScanBatcher(meta)
    new_scans = [core.LidarScan(meta)] * num_iters

    # perform the actual test
    num_batched = 0
    profile.start()
    for p in packets:
        if batcher(p, new_scans[num_batched]):
            num_batched = num_batched + 1
    profile.end(num_iters)
    assert num_batched == num_iters


# now for each scan source type
@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.performance
def test_perf_osf_save(scan: core.LidarScan, meta: core.SensorInfo, profile, tmp_path) -> None:
    # setup the test
    # save N scans to OSF and close
    from ouster.sdk.osf import Writer
    file_name = str(tmp_path / "test.osf")
    num_iters = profile.iterations(40)
    scans = []
    for i in range(num_iters):
        scans.append(copy.copy(scan))

    # perform the actual test
    profile.start()
    writer = Writer(file_name, meta)
    for scan in scans:
        writer.save(0, scan)
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
    scan = next(iter(src))[0]
    writer = Writer(fn, meta)
    count = 100 if request.config.getoption("--performance") else 2
    if request.config.getoption("--num-iterations"):
        count = int(request.config.getoption("--num-iterations"))
    for i in range(count):
        writer.save(0, scan)
        scan.frame_id = scan.frame_id + 1
        scan.packet_timestamp[:] = scan.packet_timestamp + 100000000
        scan.timestamp[:] = scan.timestamp + 100000000
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
    scan = next(iter(src))[0]
    count = 100 if request.config.getoption("--performance") else 2
    if request.config.getoption("--num-iterations"):
        count = int(request.config.getoption("--num-iterations"))
    with open(fn_json, "w") as f:
        f.write(meta.to_json_string())

    handle = _pcap.record_initialize(fn, 2**16)
    for i in range(count):
        # convert to packets and save
        for packet in scan_to_packets(scan, meta):
            _pcap.record_packet(handle, "127.0.0.1", "127.0.0.1", meta.config.udp_port_lidar,
                                meta.config.udp_port_lidar, packet.buf, packet.host_timestamp / 1e9)

        scan.frame_id = scan.frame_id + 1
        scan.packet_timestamp[:] = scan.packet_timestamp + 100000000
        scan.timestamp[:] = scan.timestamp + 100000000

    _pcap.record_uninitialize(handle)
    return fn


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.performance
def test_perf_osf_read(scan: core.LidarScan, meta: core.SensorInfo, profile, tmp_osf) -> None:
    # Read all scans in the OSF
    from ouster.sdk import open_source
    num_iters = 0

    profile.start()
    for i in open_source(tmp_osf):
        num_iters = num_iters + 1
    profile.end(num_iters)


@pytest.mark.performance
def test_perf_osf_open(profile, tmp_osf) -> None:
    # open the osf and read one scan N times
    from ouster.sdk import open_source
    num_iters = profile.iterations(40)

    profile.start()
    for i in range(num_iters):
        for scan in open_source(tmp_osf):
            break
    profile.end(num_iters)


@pytest.mark.performance
def test_perf_pcap_read(profile, tmp_pcap) -> None:
    # Read all scans in the PCAP
    from ouster.sdk import open_source
    num_iters = 0

    profile.start()
    for i in open_source(tmp_pcap):
        num_iters = num_iters + 1
    profile.end(num_iters)


@pytest.mark.performance
def test_perf_pcap_open(profile, tmp_pcap) -> None:
    # open the osf and read one scan N times
    from ouster.sdk import open_source
    num_iters = profile.iterations(40)

    profile.start()
    for i in range(num_iters):
        for scan in open_source(tmp_pcap):
            break
    profile.end(num_iters)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.performance
def test_perf_pcap_save(scan: core.LidarScan, meta: core.SensorInfo, profile, tmp_path) -> None:
    # setup the test
    # save N scans to PCAP and close
    num_iters = profile.iterations(40)

    import ouster.sdk._bindings.pcap as _pcap
    fn = str(tmp_path / "test.pcap")
    fn_json = str(tmp_path / "test.json")

    # make the scans
    scans = []
    for i in range(num_iters):
        scan2 = copy.copy(scan)
        scans.append(scan2)
        scan.frame_id = scan.frame_id + 1
        scan.packet_timestamp[:] = scan.packet_timestamp + 100000000
        scan.timestamp[:] = scan.timestamp + 100000000

    # perform the actual test
    profile.start()
    with open(fn_json, "w") as f:
        f.write(meta.to_json_string())

    handle = _pcap.record_initialize(fn, 2**16)
    for scan in scans:
        # convert to packets and save
        for packet in scan_to_packets(scan, meta):
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
    # slice the file to the last scan and run stats on it
    from ouster.sdk import open_source
    l = len(open_source(tmp_osf))

    num_iters = 1
    slice = str(l - 1) + ":"
    profile.start()
    result = runner.invoke(cli_core.cli, ['source', tmp_osf, 'slice', slice, 'stats'])  # type: ignore
    profile.end(num_iters)
    print(result.output)
    assert result.exit_code == 0
