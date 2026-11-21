from ouster.sdk.examples.voxel_downsample import voxel_downsample_example
from ouster.sdk.examples.voxel_hash_map import voxel_hash_map_example


def test_voxel_hash_map_example(capsys):
    voxel_hash_map_example()
    out = capsys.readouterr().out
    assert "layout stride: 17 bytes/point" in out
    assert "attribute dtypes: uint32, uint8, float32" in out


def test_voxel_downsample_example(capsys):
    voxel_downsample_example()
    out = capsys.readouterr().out
    assert "attribute dtypes: uint32, uint8, float32" in out
    assert "attribute shapes: (" in out
    assert "AVERAGE_POINT:" in out
