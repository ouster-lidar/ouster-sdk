from contextlib import closing
from copy import copy
import logging

from more_itertools import flatten, take
import numpy as np
import pytest

from ouster.sdk import client
from ouster.sdk.client import LidarMode

pytestmark = pytest.mark.usefixtures("hil_initial_config")

logger = logging.getLogger("HIL")

MODES = [
    (LidarMode.MODE_512x20, 512), (LidarMode.MODE_1024x10, 1024), (LidarMode.MODE_2048x10, 2048),
    (LidarMode.MODE_4096x5, 4096)
]

def genparams(modes):
    """Generate some azimuth window cases for various lidar modes."""

    def mkparam(mode, window, *, expect_fail=False):
        return pytest.param(
            (mode, window, expect_fail),
            id=f"{mode}-{window[0]}:{window[1]}{'-fail' if expect_fail else ''}"
        )

    def genwindows(info):
        # centidegrees between beams, rounded down
        mode = info[0]
        cols = info[1]
        bi = 360000 // cols
        return [
            # 0 is always one of the exact angles, so it's always at least one
            # column selected [all ranges are inclusive]
            mkparam(mode, (0, 0)),
            mkparam(mode, (1, 1), expect_fail=True),
            mkparam(mode, (1, 2), expect_fail=True),
            mkparam(mode, (0, bi)),
            mkparam(mode, (0, bi + 1)),
            mkparam(mode, (bi, bi * 2 + 1)),
            mkparam(mode, (0, 360000)),
            mkparam(mode, (1, 360000)),
            mkparam(mode, (bi, 360000)),
            mkparam(mode, (bi + 1, 360000)),
            mkparam(mode, (180000, 180000 + bi)),
            mkparam(mode, (180001, 180001 + bi)),
            mkparam(mode, (180001, 180002), expect_fail=True),
            mkparam(mode, (180000, 270000)),
            mkparam(mode, (180000, 269999)),
            mkparam(mode, (180001, 270000)),
            mkparam(mode, (180001, 269999)),
            mkparam(mode, (180000, 360000)),
            mkparam(mode, (180000, 359999)),
            mkparam(mode, (180001, 360000)),
            mkparam(mode, (180001, 359999)),
            mkparam(mode, (270000, 0)),
            mkparam(mode, (270000, 1)),
            mkparam(mode, (270000, bi)),
            mkparam(mode, (270000, bi + 1)),
            mkparam(mode, (360000, 0)),
        ]

    return flatten(map(genwindows, modes))


@pytest.fixture(params=genparams(MODES))
def azimuth_params(hil_initial_config, request):
    return request.param


@pytest.fixture
def hil_sensor_config(hil_initial_config,
                      azimuth_params) -> client.SensorConfig:
    mode, window, _ = azimuth_params

    newcfg = copy(hil_initial_config)
    newcfg.lidar_mode = mode

    newcfg.azimuth_window = window
    return newcfg


def test_azimuth_setting(hil_configured_sensor, hil_sensor_config,
                         azimuth_params):
    """Test that the sensor sends the reported column window.

    TODO: This does not sanity-check how the column window is computed from the
    azimuth window.
    """

    _, _, expect_fail = azimuth_params

    if hil_configured_sensor is None:
        assert expect_fail, "Configuration failed unexpectedly"
        logger.debug("Not configured, skipping...")
        return
    else:
        assert not expect_fail, "Configuration succeeded unexpectedly"

    with closing(client.Scans.stream(hil_configured_sensor,
                                     complete=False)) as scans:
        w = scans.metadata.format.columns_per_frame

        col_window = scans.metadata.format.column_window
        logger.debug(f"Reported col_window: {col_window}")

        window_len = (col_window[1] - col_window[0] + w) % w + 1
        logger.debug(f"Expected valid columns: {window_len}")

        ss = take(10, scans)

    for s in ss:
        if not s.complete(col_window):
            logger.warning(f"Received incomplete frame {s.frame_id}: "
                           f"expected {window_len} valid cols but got "
                           f"{np.count_nonzero(s.status & 0x1)}")

    assert any(
        window_len == np.count_nonzero(s.status & 0x1)
        for s in ss), "Number of valid columns doesn't match window length"

    assert any(s.complete(col_window)
               for s in ss), "Received no complete frames"
