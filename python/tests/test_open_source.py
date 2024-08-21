import os
from re import escape
import pytest
import tempfile
import ouster.sdk.io_type
from ouster.sdk.util import resolve_metadata_multi
from ouster.sdk import open_source
from tests.conftest import PCAPS_DATA_DIR


def test_open_source_empty_source_url():
    """It should raise an error if the src url is the empty string."""
    with pytest.raises(ValueError, match="No valid source specified"):
        open_source('')


def test_open_source_multi_source_url():
    """It should raise an error if the src url contains commas.
    """
    with pytest.raises(NotImplementedError, match="providing more than a single url is current not supported!"):
        open_source('a,b')


def test_open_source_unsupported_source_type():
    """It raises a NotImplementedError if the source type is not supported."""
    with tempfile.NamedTemporaryFile(suffix='.csv') as f:
        with pytest.raises(NotImplementedError, match="The io_type:OusterIoType.CSV is not supported!"):
            open_source(f.name)


def test_open_source_undetermined_source_type():
    """It raises a RuntimeError if the source type couldn't be determined."""
    with pytest.raises(RuntimeError, match=escape("Failed to create scan_source for url ['unknown source']\n "
        "more details: Source type expected to be a sensor hostname, ip address, or a .pcap, .osf, or .bag file.")):
        open_source('unknown source')


def test_open_source_meta_not_supported(monkeypatch):
    """It raises a RuntimeError if the meta keyword is provided to an unsupported source type."""
    with pytest.raises(RuntimeError, match="SensorScanSource does not support user-supplied metadata."):
        # monkeypatch io_type to return a OusterIoType.SENSOR
        with monkeypatch.context() as m:
            m.setattr(ouster.sdk.io_type, "io_type", lambda _: ouster.sdk.io_type.OusterIoType.SENSOR)
            open_source('fakesensor', meta='fake_meta.json')

    with pytest.raises(RuntimeError, match="OsfScanSource does not support user-supplied metadata."):
        # monkeypatch io_type to return a OusterIoType.SENSOR
        with monkeypatch.context() as m:
            m.setattr(ouster.sdk.io_type, "io_type", lambda _: ouster.sdk.io_type.OusterIoType.OSF)
            open_source('fake.osf', meta='fake_meta.json')


def test_open_source_meta_pcap():
    """Providing the meta parameter to open source should override the metadata files used by the PcapScanSource."""
    pcap_file_path = os.path.join(PCAPS_DATA_DIR, 'VLI-16-one-packet.pcap')
    json_file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.json')

    # the test file is different than what would be resolved ordinarily
    assert json_file_path not in resolve_metadata_multi(pcap_file_path)
    src = open_source(pcap_file_path, sensor_idx=-1, meta=(json_file_path,))
    # TODO: src._metadata_paths is not a standard field
    assert src._metadata_paths == [json_file_path]
