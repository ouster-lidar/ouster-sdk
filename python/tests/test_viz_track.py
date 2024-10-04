from ouster.sdk.client import LidarScan, first_valid_column_pose, SensorInfo, LidarMode
from ouster.sdk.viz.model import LidarScanVizModel
from ouster.sdk.viz.track import Track, MultiTrack, TRACK_INIT_POINTS_NUM, TRACK_MAP_GROWTH_RATE
from ouster.sdk.viz.accumulators_config import LidarScanVizAccumulatorsConfig


def test_track_init_defaults():
    """It should set some defaults."""
    config = LidarScanVizAccumulatorsConfig()
    track = Track(config)
    assert track._xyz.shape[0] == 100
    assert track._key.shape[0] == 100
    assert track._kf_xyz.shape[0] == 1
    assert track._kf_key.shape[0] == 1
    assert len(track._scan_records) == 0


def test_track_update():
    """Calling update with a new scan adds the scan to the scan records
    list."""
    config = LidarScanVizAccumulatorsConfig()
    track = Track(config)
    scan = LidarScan(1, 1)
    assert len(track._scan_records) == 0
    assert track._scan_num == -1
    track.update(scan, 0)
    assert len(track._scan_records) == 1
    assert track._scan_num == 0
    track.update(scan, 1)
    assert len(track._scan_records) == 2
    assert track._scan_num == 1
    track.update(scan, 2)
    assert len(track._scan_records) == 3
    assert track._scan_num == 2


def test_track_update_2():
    """Calling update with a previously-seen scan does not add it to the scan records
    list."""
    config = LidarScanVizAccumulatorsConfig()
    track = Track(config)
    scan = LidarScan(1, 1)
    assert len(track._scan_records) == 0
    assert track._scan_num == -1
    track.update(scan, 0)
    assert len(track._scan_records) == 1
    assert track._scan_num == 0
    track.update(scan, 1)
    assert len(track._scan_records) == 2
    assert track._scan_num == 1

    track.update(scan, 0)
    assert len(track._scan_records) == 2
    assert track._scan_num == 0


def test_track_update_3():
    """It should expand the list of points used for the track."""
    # TODO[tws]: we should either use fixed-sized data structures, or we should encapsulate the growth and overflow
    # behavior of the data structures.  Mixing the growth logic with the accumulator business logic is an example of
    # unnecessary coupling.
    config = LidarScanVizAccumulatorsConfig()
    track = Track(config)

    # preconditions
    assert track._xyz.shape[0] == TRACK_INIT_POINTS_NUM
    assert track._key.shape[0] == TRACK_INIT_POINTS_NUM
    assert track._track_idx == 0
    assert len(track._scan_records) == 0

    # add enough scans to fill up the track
    scan = LidarScan(1, 1)
    for scan_num in range(TRACK_INIT_POINTS_NUM):
        track.update(scan, scan_num)

    assert track._xyz.shape[0] == TRACK_INIT_POINTS_NUM
    assert track._track_idx == TRACK_INIT_POINTS_NUM

    # add one more
    track.update(scan, scan_num)

    new_size = TRACK_INIT_POINTS_NUM * TRACK_MAP_GROWTH_RATE + 1
    assert track._xyz.shape[0] == new_size
    assert track._key.shape[0] == new_size
    assert track._track_idx == TRACK_INIT_POINTS_NUM + 1


def test_track_update_4():
    """It does not update records for scans that have already been logged (according to scan_num!)."""
    config = LidarScanVizAccumulatorsConfig(accum_max_num=100)
    track = Track(config)
    scan = LidarScan(1, 1)
    scan2 = LidarScan(1, 1)
    track.update(scan, 0)
    assert track._scan_records[0].scan == scan
    track.update(scan2, 0)
    assert track._scan_records[0].scan == scan


def test_track_update_5():
    """It only creates ScanRecords for key frames(?).
    This reduces the amount of memory retained for ScansAccumulator cloud color channels (aka cloud_mode_keys)."""
    # TODO[tws] describe this behavior a little better
    config = LidarScanVizAccumulatorsConfig(accum_max_num=0)
    track = Track(config)
    scan = LidarScan(1, 1)
    scan2 = LidarScan(1, 1)
    track.update(scan, 0)
    assert track._scan_records[0] is None
    track.update(scan2, 0)
    assert track._scan_records[0] is None


def test_multitrack_update():
    """It updates a track for each sensor."""
    infos = [
        SensorInfo.from_default(LidarMode.MODE_2048x10),
        SensorInfo.from_default(LidarMode.MODE_2048x10)
    ]
    model = LidarScanVizModel(infos, _img_aspect_ratio=0)
    config = LidarScanVizAccumulatorsConfig(accum_max_num=2, accum_min_dist_num=2)
    multitrack = MultiTrack(model, config)
    assert len(multitrack._tracks) == len(infos)

    # create a scan for each sensor
    scans = [LidarScan(1, 1) for _ in infos]

    # update the tracks with the scans
    multitrack.update(scans, 0)
    for track, scan in zip(multitrack._tracks, scans):
        assert track._scan_records[0].scan == scan

    # omitting scan_num increases scan_num by 1
    multitrack.update(scans)
    for track, scan in zip(multitrack._tracks, scans):
        assert track._scan_records[1].scan == scan


def test_key_frames():
    """It should add a key frame every "accum_min_dist_num" scans."""
    config = LidarScanVizAccumulatorsConfig(accum_max_num=2, accum_min_dist_num=2)
    track = Track(config)

    # add enough scans to fill up the track
    scans_to_add = 4
    scan = LidarScan(1, 1)
    for scan_num in range(scans_to_add):
        track.update(scan, scan_num)

    assert track._scan_num == scans_to_add - 1
    assert len(track._scan_records) == scans_to_add
    assert track._key_frames == [0, 2, None]

    track.update(scan, track._scan_num + 1)
    assert len(track._scan_records) == scans_to_add + 1
    assert track._key_frames == [None, 2, 4]

    track.update(scan, track._scan_num + 1)
    track.update(scan, track._scan_num + 1)
    assert len(track._scan_records) == scans_to_add + 3
    assert track._key_frames == [6, None, 4]


def test_key_frames_2():
    """It should add a key frame every "accum_min_dist_meters"."""
    config = LidarScanVizAccumulatorsConfig(accum_max_num=2, accum_min_dist_num=0, accum_min_dist_meters=2)
    track = Track(config)

    def scan_with_xyz(x, y, z):
        scan = LidarScan(1, 1)
        pose = first_valid_column_pose(scan)
        pose[0, 3] = x
        pose[1, 3] = y
        pose[2, 3] = z
        return scan

    track.update(scan_with_xyz(0, 0, 0), 0)
    assert track._key_frames == [0, None, None]

    # This point isn't far enough to be a key frame
    track.update(scan_with_xyz(1, 0, 0), 1)
    assert track._key_frames == [0, None, None]

    # This point is far enough to be a key frame
    track.update(scan_with_xyz(config._accum_min_dist_meters + 1, 0, 0), 2)
    assert track._key_frames == [0, 2, None]
