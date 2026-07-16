import pytest
from ouster.sdk.mapping import LIOSlamConfig, SlamConfig, SlamEngine
from ouster.sdk.open_source import open_source


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


def test_slam_update_skips_frame_with_no_valid_columns(input_osf_file):
    """A frame with no valid columns must be skipped, not crash update().

    Regression test for OSDK-1156: flagging every column invalid (zeroed status)
    used to make get_first_valid_column() throw deep in the update path,
    propagating an uncaught RuntimeError out of slam_engine.update().
    """
    source = open_source(str(input_osf_file))
    config = SlamConfig.create("lio")
    config.min_range = 1.0
    config.max_range = 100.0
    config.voxel_size = 0.5  # set so update() doesn't estimate it
    engine = SlamEngine.create(source.sensor_info, config)

    count = 0
    for i, frame_set in enumerate(source):
        if i == 0:
            # flag every column invalid -> zero valid columns
            frame_set[0].status[:] = 0
        engine.update(frame_set)  # must not raise "No valid columns in LidarFrame"
        count += 1

    assert count > 0


def test_slam_engine_initialization(input_osf_file):
    """It should initialize without error with a valid SensorInfo and config."""
    source = open_source(str(input_osf_file))
    config = SlamConfig.create("lio")
    config.min_range = 1.0
    config.max_range = 100.0
    # Verification that create method exists and works
    engine = SlamEngine.create(source.sensor_info, config)
    assert engine is not None


def test_slam_engine_default_config(input_osf_file):
    """SlamEngine.create(sensor_info) should use a default LIOSlamConfig."""
    source = open_source(str(input_osf_file))
    engine = SlamEngine.create(source.sensor_info)
    assert engine is not None


def test_slam_config_factory_default_kind():
    """SlamConfig.create() with no args should return an LIOSlamConfig."""
    config = SlamConfig.create()
    assert isinstance(config, LIOSlamConfig)


def test_slam_config_factory_explicit_kind():
    """SlamConfig.create(kind='lio') should return an LIOSlamConfig."""
    config = SlamConfig.create(kind="lio")
    assert isinstance(config, LIOSlamConfig)


def test_slam_config_fields_settable():
    """Fields on a created config should be settable and readable."""
    config = SlamConfig.create()
    config.min_range = 0.5
    config.max_range = 80.0
    assert config.min_range == 0.5
    assert config.max_range == 80.0


def test_slam_config_factory_wrong_type():
    """It should throw when given an unsupported type."""
    with pytest.raises(ValueError, match="Unsupported SlamConfig type"):
        SlamConfig.create("doesntexist")
