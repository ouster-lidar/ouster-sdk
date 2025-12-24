import numpy as np
from ouster.sdk._bindings.client import Mesh, Triangle


def test_mesh(test_data_dir):
    stl_path = test_data_dir / "zone_monitor/ascii.stl"
    m = Mesh()
    assert m.load_from_stl(str(stl_path))
    assert len(m.triangles) == 12
    tri = m.triangles[0]
    assert np.array_equal(tri.normal, (0.0, 0.0, 1.0))
    assert np.array_equal(tri.coords[0], (-20.0, -20.0, 40.0))


def test_mesh_bindings_2():
    m = Mesh([
        Triangle(
                np.array([0.0, 0.0, 0.0]),
                np.array([1.0, 0.0, 0.0]),
                np.array([0.0, 1.0, 0.0]),
        ),
        Triangle(
                np.array([0.0, 0.0, 0.0]),
                np.array([0.0, 1.0, 0.0]),
                np.array([0.0, 0.0, 1.0]),
        ),
    ])

    assert len(m.triangles) == 2


def test_mesh_bindings_3(tmp_path):
    m = Mesh()
    with open(tmp_path / "test_zone_0.stl", "w") as f:
        f.write(
            """solid test_zone_0
facet normal 0 0 0
outer loop
vertex 0 0 0
vertex 1 0 0
vertex 0 1 0
endloop
endfacet
endsolid test_zone_0
"""
        )
    assert m.load_from_stl(f.name) is True
    assert len(m.triangles) == 1
