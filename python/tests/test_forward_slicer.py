import pytest
from ouster.sdk.util.forward_slicer import ForwardSlicer


class ReferenceSliceable:
    def __init__(self, data):
        self.data = data

    def __len__(self):
        return len(self.data)

    def __getitem__(self, key):
        return self.data[key]


class NormalizedSliceable:

    def __init__(self, data):
        self.data = data

    def __len__(self):
        return len(self.data)

    def __getitem__(self, key):
        if isinstance(key, slice):
            k = ForwardSlicer.normalize(key, len(self))
            result = ForwardSlicer.slice(iter(self.data[k.start:k.stop]), k)
            return result if k.step > 0 else list(reversed(result))
        else:
            return self.data[key]


@pytest.fixture
def sliceable_fixture():
    values = [i for i in range(10)]
    ref_sliceable = ReferenceSliceable(values)
    test_sliceable = NormalizedSliceable(values)
    return ref_sliceable, test_sliceable


@pytest.mark.parametrize("start, stop, step", [
    (None, None, None),
    (None, 8, None),
    (3, None, None),
    (3, 8, None),
    (3, 8, 1),
    (3, 8, 2),
    (3, 8, 3),
    (0, 8, 1),
    (3, 10, 1),
    (0, 10, 1),
    (0, 10, 2),
    (0, 10, 3),
    (0, 10, 5),
    (3, 11, 1),
    (3, -1, 1),
    (3, -2, 1),
    (-7, 8, 1),
    (-7, -2, 1),
    (8, 3, 1),      # invalid range for positive step
    (3, 8, -1),     # invalid range for negative step
    (8, 3, -1),
    (8, 3, -2),
    (8, 3, -3),
    (8, 3, -4),
    (5, 3, -3),
    (7, 3, -2),
    (8, 0, -1),
    (10, 3, -1),
    (10, 0, -1),
    (11, 3, -1),
    (-1, 3, -1),
    (-2, 3, -1),
    (8, -7, -1),
    (-2, -7, -1),
    (-1, -7, -1),
    (-1, 0, -1),
    (-1, 0, -2),
    (-1, 0, -3),
    (9, 0, -1),
    (9, 0, -2),
    (9, 0, -3),
    (10, 0, -1),
    (10, 0, -2),
    (10, 0, -3),
    (11, 0, -1),
    (11, 0, -2),
    (11, 0, -3),
    (9, None, -1),
    (9, None, -2),
    (9, None, -3),
    (None, 3, -1),
    (None, 3, -2),
    (None, 3, -3),
    (None, 0, -1),
    (None, 0, -2),
    (None, 0, -3),
    (None, None, -1),
    (None, None, -2),
    (None, None, -3),
])
def test_sliceable(sliceable_fixture, start, stop, step):
    ref_sliceable, test_sliceable = sliceable_fixture
    ss = slice(start, stop, step)
    assert ref_sliceable[ss] == test_sliceable[ss], f"Failed test case with the slice [{start}:{stop}:{step}]"
