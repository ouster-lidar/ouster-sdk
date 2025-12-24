import numpy as np

from ouster.sdk.examples import colormaps


def test_colorize_uses_spezia_indices():
    """It should map normalized inputs to the expected spezia rows."""
    img = np.array([[0.0, 0.5], [1.0, 0.25]], dtype=np.float64)
    result = colormaps.colorize(img)

    # indices used: 0, 127, 255, 63
    expected = colormaps.spezia[[0, 127, 255, 63]].reshape(2, 2, 3)
    assert result.shape == (2, 2, 3)
    assert result.dtype == colormaps.spezia.dtype
    assert np.allclose(result, expected)


def test_normalize_clamps_and_scales_percentiles():
    """Normalize should clamp extremes and scale to [0,1]."""
    data = np.array([-10, 0, 5, 10, 20, 30, 100], dtype=np.float64)
    normalized = colormaps.normalize(data, percentile=0.1)

    assert normalized.min() >= 0
    assert normalized.max() <= 1
    # middle values should be scaled between 0 and 1
    assert 0 < normalized[2] < 1
    # extremes should clamp
    assert normalized[0] == 0
    assert normalized[-1] == 1
