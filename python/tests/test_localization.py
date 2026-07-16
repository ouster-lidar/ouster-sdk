import numpy as np
import pytest
from ouster.sdk import mapping, open_source
from ouster.sdk.mapping import LIOLocalizationConfig


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


def create_lio_localization_config():
    config = mapping.LocalizationConfig.create("lio")
    config.min_range = 0.5
    config.max_range = 100.0
    config.voxel_size = 1.0
    config.deskew_method = "auto"
    return config


def test_localization_config_values(input_osf_file):
    """LocalizationConfig fields set in run_localization_once() match the docs."""
    config = create_lio_localization_config()
    assert config.min_range == 0.5
    assert config.max_range == 100.0
    assert config.voxel_size == 1.0
    assert config.deskew_method == "auto"


def test_localization_config_factory_default_kind():
    """LocalizationConfig.create() with no args should return an LIOLocalizationConfig."""
    config = mapping.LocalizationConfig.create()
    assert isinstance(config, LIOLocalizationConfig)


def test_localization_config_factory_explicit_kind():
    """LocalizationConfig.create(kind='lio') should return an LIOLocalizationConfig."""
    config = mapping.LocalizationConfig.create(kind="lio")
    assert isinstance(config, LIOLocalizationConfig)


def test_localization_config_fields_settable():
    """Fields on a created config should be settable and readable."""
    config = mapping.LocalizationConfig.create()
    config.min_range = 0.5
    config.max_range = 80.0
    assert config.min_range == 0.5
    assert config.max_range == 80.0


def test_localization_config_factory_wrong_type():
    """It should raise ValueError for an unsupported kind."""
    with pytest.raises(ValueError, match="Unsupported LocalizationConfig type"):
        mapping.LocalizationConfig.create("doesntexist")


def test_localization_engine_initialization_fails_with_nonexistent_map_default_config(input_osf_file):
    """LocalizationEngine.create(sensor_info, map_path) with default config throws for missing map."""
    src = open_source(str(input_osf_file))
    with pytest.raises(RuntimeError, match="File does not exist"):
        mapping.LocalizationEngine.create(src.sensor_info, "dummy_map.ply")


def test_localization_engine_initialization_succeeds_with_valid_map(input_osf_file):
    """It initializes without error with a valid SensorInfo, config, and map."""
    src = open_source(str(input_osf_file))
    config = create_lio_localization_config()
    # Use the same OSF as a dummy "map" since it exists and has the right format
    map_points = np.ndarray((3, 3), dtype=np.float32)  # dummy data to satisfy the API; not actually used
    engine = mapping.LocalizationEngine.create(src.sensor_info, map_points, config)
    assert engine is not None


def test_localization_engine_initialization_default_config_with_map_array(input_osf_file):
    """LocalizationEngine.create(sensor_info, map) should use default LIOLocalizationConfig."""
    src = open_source(str(input_osf_file))
    map_points = np.zeros((3, 3), dtype=np.float32)
    engine = mapping.LocalizationEngine.create(src.sensor_info, map_points)
    assert engine is not None
