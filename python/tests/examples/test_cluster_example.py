import sys
import pytest
from ouster.sdk.examples.cluster_example import add_objects_to_source


@pytest.mark.perception
@pytest.mark.skipif(sys.platform == "win32", reason="Does not run on Windows")
def test_cluster_example(test_data_dir, capsys):
    source_url = test_data_dir / "osfs" / "single_scan_016.osf"
    add_objects_to_source(str(source_url))
    captured = capsys.readouterr()
    assert 'Key classic has 170 objects.' in captured.out
