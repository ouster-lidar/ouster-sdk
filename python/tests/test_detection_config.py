"""
Copyright (c) 2026, Ouster, Inc.
All rights reserved.
"""

import warnings

import pytest

from ouster.sdk.open_source import open_source
from ouster.sdk.perception import (
    ClassicDetectionConfig,
    DetectionConfig,
    DetectionEngine,
)


def test_create_detection_config_default_kind():
    cfg = DetectionConfig.create()
    assert isinstance(cfg, ClassicDetectionConfig)
    assert cfg.save_instance_id_fields is True


@pytest.mark.parametrize("kind", ["classic", "CLASSIC", "Classic"])
def test_create_detection_config_kind_case_insensitive(kind: str):
    cfg = DetectionConfig.create(kind)
    assert isinstance(cfg, ClassicDetectionConfig)


def test_create_detection_config_applies_base_fields():
    cfg = DetectionConfig.create("classic", save_instance_id_fields=False)
    assert cfg.save_instance_id_fields is False


def test_create_detection_config_applies_classic_fields():
    cfg = DetectionConfig.create(
        "classic",
        cluster_filter_min_side_length=1.25,
        cluster_filter_min_vertical_size=2.5,
        cluster_filter_max_volume=100.0,
        cluster_filter_max_side_length=3.0,
    )
    assert cfg.cluster_filter_min_side_length == 1.25
    assert cfg.cluster_filter_min_vertical_size == 2.5
    assert cfg.cluster_filter_max_volume == 100.0
    assert cfg.cluster_filter_max_side_length == 3.0


def test_create_detection_config_unknown_kind():
    with pytest.raises(ValueError, match="Unknown detection engine kind"):
        DetectionConfig.create("not_a_real_engine")


def test_create_detection_config_unknown_kwarg_warns():
    with pytest.warns(UserWarning, match=r"no config field 'typo'"):
        cfg = DetectionConfig.create("classic", typo=123)
    assert isinstance(cfg, ClassicDetectionConfig)
    assert not hasattr(cfg, "typo")


def test_create_detection_config_mixed_kwargs_warns_for_unknown_only():
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        cfg = DetectionConfig.create(
            "classic",
            cluster_filter_max_volume=7.0,
            unknown_param=True,
        )
    assert cfg.cluster_filter_max_volume == 7.0
    assert len(w) == 1
    assert "unknown_param" in str(w[0].message)


def test_create_detection_config_multiple_unknown_kwargs_emit_multiple_warnings():
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        DetectionConfig.create("classic", a=1, b=2)
    assert len(w) == 2


# ---------------------------------------------------------------------------
# DetectionEngine.create overloads
# ---------------------------------------------------------------------------


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


@pytest.mark.perception
def test_detection_engine_create_default_config(input_osf_file):
    """DetectionEngine.create(sensor_infos) should use a default ClassicDetectionConfig."""
    source = open_source(str(input_osf_file))
    engine = DetectionEngine.create(source.sensor_info)
    assert engine is not None


@pytest.mark.perception
def test_detection_engine_create_with_classic_config(input_osf_file):
    """DetectionEngine.create(sensor_infos, config) should accept a ClassicDetectionConfig."""
    source = open_source(str(input_osf_file))
    config = ClassicDetectionConfig()
    config.cluster_filter_min_side_length = 0.5
    engine = DetectionEngine.create(source.sensor_info, config)
    assert engine is not None


@pytest.mark.perception
def test_detection_engine_create_with_kind_string(input_osf_file):
    """DetectionEngine.create(sensor_infos, kind) should accept a kind string."""
    source = open_source(str(input_osf_file))
    engine = DetectionEngine.create(source.sensor_info, "classic")
    assert engine is not None


@pytest.mark.perception
def test_detection_engine_create_invalid_kind(input_osf_file):
    """DetectionEngine.create with an unknown kind string should raise ValueError."""
    source = open_source(str(input_osf_file))
    with pytest.raises(ValueError, match="Unknown detection engine kind"):
        DetectionEngine.create(source.sensor_info, "not_a_real_engine")
