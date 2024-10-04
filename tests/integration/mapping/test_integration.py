# Please note that the tests in `tests/integration` folder cannot be run on your system
# as they use external data (not included in the repository).

import glob
import os
import shlex
import subprocess
import sys
import re
import ouster.sdk.mapping
from pathlib import Path

import pytest
import platform

# Check if the platform is macOS and if it's running on ARM architecture
is_mac_arm = platform.system() == 'Darwin' and platform.processor() == 'arm'

@pytest.fixture
def verify_kiss_icp():
    try:
        import kiss_icp   # type: ignore
        return True
    except BaseException:
        return False


@pytest.fixture
def verify_point_cloud_util():
    try:
        import point_cloud_utils   # type: ignore
        return True
    except BaseException:
        return False


@pytest.fixture
def pcap_full_path():
    data_path = os.getenv('TEST_DATA_DIR', None)
    data_path = os.path.join(data_path, "mapping")
    return os.path.join(data_path , 'short.pcap').replace("\\", "/")


@pytest.fixture
@pytest.mark.skipif(is_mac_arm, reason="Skipping tests on MacBook with ARM architecture")
def slam_conversion_path(pcap_full_path, tmp_path):
    slam_out_osf = os.path.join(tmp_path, "out.osf").replace("\\", "/")
    execution_command = f"ouster-cli source {pcap_full_path} slam save {slam_out_osf}"
    process_slam = subprocess.run(
        shlex.split(execution_command),
        text=True,
        capture_output=True)
    assert process_slam.returncode == 0, process_slam.stderr
    assert os.path.exists(slam_out_osf)
    return slam_out_osf

try:
    from ouster.sdk.mapping import KissBackend
    kiss_icp_available = True
except ImportError:
    kiss_icp_available = False

@pytest.mark.skipif(not kiss_icp_available or is_mac_arm, reason="No Platform Support or on mac arm")
def test_slam_source_help_command(pcap_full_path):
    # Test ouster-cli source --help
    help_command_1 = "ouster-cli source --help"
    process_help_1 = subprocess.run(
        shlex.split(help_command_1),
        text=True,
        capture_output=True)
    assert process_help_1.returncode == 0, process_help_1.stderr

    # Test ouster-cli source A.pcap --help
    help_command_2 = f"ouster-cli source {pcap_full_path} --help"
    process_help_2 = subprocess.run(
        shlex.split(help_command_2),
        text=True,
        capture_output=True)
    assert process_help_2.returncode == 0, process_help_2.stderr

    # Test ouster-cli source A.pcap save --help
    help_command_3 = f"ouster-cli source {pcap_full_path} save --help"
    process_help_3 = subprocess.run(
        shlex.split(help_command_3),
        text=True,
        capture_output=True)
    assert process_help_3.returncode == 0, process_help_3.stderr


def convert(input_file, output_file):

    temp_path = Path(output_file)
    extension = temp_path.suffix
    name = temp_path.stem

    files = glob.glob(os.path.join(temp_path.parents[0], f"{name}*.{extension}"))
    assert len(files) == 0

    input_file = input_file.replace("\\", "/")
    output_file = output_file.replace("\\", "/")
    convert_command = f"ouster-cli source {input_file} save {output_file}"
    process_convert = subprocess.run(shlex.split(convert_command),
                                     text=True,
                                     capture_output=True)
    assert process_convert.returncode == 0

    print(temp_path.parents[0])
    print(os.listdir(temp_path.parents[0]))
    check = os.path.join(temp_path.parents[0], f"{name}*{extension}")
    print(check)
    files = glob.glob(check)
    print(files)
    assert len(files) == 1

    return files[0].replace("\\", "/")

@pytest.mark.skipif(not kiss_icp_available or is_mac_arm, reason="No Platform Support or on mac arm")
@pytest.mark.parametrize("output_file,min_size", [
    ("test_output.osf", 1000),
    ("test_output.ply", 1000),
    ("test_output.las", 1000),
    ("test_output.pcd", 500)])
def test_slam_command_output(output_file, min_size,
                             slam_conversion_path,
                             tmp_path):
    output_file = os.path.join(tmp_path, output_file)
    output_file = convert(slam_conversion_path, output_file)

    assert os.path.exists(output_file)
    file_size = os.path.getsize(output_file) // (1024)
    assert file_size >= min_size
