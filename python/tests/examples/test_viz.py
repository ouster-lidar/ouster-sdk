# type: ignore
import pytest
from ouster.sdk.examples import viz
from ouster.sdk import open_source
from pathlib import Path

import sys

# TODO fix this ugly hack
sys.path.append(str(Path(__file__).resolve().parent.parent))
from conftest import MockPointViz   # noqa: E402


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


def test_viz_example(input_osf_file):
    """it shouldn't throw any errors when running the examples"""
    source = open_source(str(input_osf_file))
    meta = source.sensor_info[0]
    frames = iter(source)
    frame = next(frames)[0]

    mock_viz = MockPointViz()
    viz.example_1_2(mock_viz, meta, iter(source))
    viz.example_2_0(mock_viz, meta, frame)
    viz.example_2_1(mock_viz, meta, frame)
    viz.example_2_2(mock_viz, meta, frame)
    viz.example_2_3(mock_viz, meta, frame)
    viz.example_3(mock_viz)
    viz.example_4(mock_viz)
    viz.example_5(mock_viz)
