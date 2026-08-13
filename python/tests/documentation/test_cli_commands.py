"""CLI doc-snippet tests for documented ouster-cli commands.
Run:
    pytest python/tests/documentation/test_cli_commands.py -v

With a live sensor:
    SENSOR_HOSTNAME=os-xxxx.local pytest python/tests/documentation/test_cli_commands.py -v
"""

import gc
import os
import shlex
import shutil
import subprocess
from pathlib import Path
from typing import Optional, Tuple
from unittest.mock import patch

import pytest
from click.testing import CliRunner

from ouster.cli import core
from ouster.cli.plugins import source  # noqa: F401
from ouster.cli.plugins import source_sensor  # noqa: F401
from ouster.cli.plugins import discover as discover_plugin  # noqa: F401
from ouster.cli.plugins import source_replay  # noqa: F401
from ouster.cli.plugins import source_mapping   # noqa: F401
from ouster.cli.plugins import source_localization  # noqa: F401
from ouster.cli.plugins import map_export  # noqa: F401
from ouster.cli.plugins.perception import source_detect  # noqa: F401
from ouster.sdk import core as sdk_core
from ouster.sdk import open_source as sdk_open_source

REPO_ROOT = Path(__file__).resolve().parents[3]
TESTS_DIR = REPO_ROOT / "tests"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _remove_comments_and_whitespace(text: str) -> str:
    """Strip doc-tag comment lines and blank lines; join continuations."""
    parts = []
    for line in text.strip().splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped.endswith("\\"):
            parts.append(stripped[:-1].strip())
        else:
            parts.append(stripped)
    return " ".join(parts)


def command_to_args(command_text: str) -> Tuple[list, Optional[str]]:
    """Convert a raw command f-string into a Click-compatible arg list.

    Strips the leading ouster-cli token, handles shell > redirects,
    and tokenizes with shlex.split.

    Returns (args, redirect_path) where redirect_path is the path
    after > if present, or None.
    """
    full = _remove_comments_and_whitespace(command_text)
    redirect_path = None
    if ">" in full:
        before, _, after = full.partition(">")
        full = before.strip()
        redirect_path = after.strip().strip('"').strip("'")
    tokens = shlex.split(full.replace("\\", "/"))
    if tokens and tokens[0] == "ouster-cli":
        tokens = tokens[1:]
    return tokens, redirect_path


def _ouster_cli_path() -> str:
    p = shutil.which("ouster-cli")
    if not p:
        pytest.fail(
            "ouster-cli not on PATH; install the package in this environment "
        )
    return p


def _run_ouster_cli_subprocess_timeout(args: list, run_for_sec: float) -> str:
    """Run command in a child process; expect subprocess.TimeoutExpired"""

    def _captured_to_str(s) -> str:
        if s is None:
            return ""
        if isinstance(s, bytes):
            return s.decode("utf-8", errors="replace")
        return str(s)

    cmd = [_ouster_cli_path(), *args]
    with pytest.raises(subprocess.TimeoutExpired) as ctx:
        subprocess.run(
            cmd,
            timeout=run_for_sec,
            capture_output=True,
            text=True,
        )
    e = ctx.value
    return _captured_to_str(e.stdout) + _captured_to_str(e.stderr)


def run_multiline_command(command: str, subs: dict, runner, cli):
    """Parse each non-comment line, substitute placeholders, and invoke."""
    for line in command.strip().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        args, _ = command_to_args(line)
        for token, replacement in subs.items():
            if token in args:
                args[args.index(token)] = replacement
        result = runner.invoke(cli, args)
        assert result.exit_code == 0, f"Output:\n{result.output}"


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def gc_after_test():
    """Force GC after every test to release viz objects before the next test.
    """
    yield
    gc.collect()


@pytest.fixture
def runner():
    return CliRunner(mix_stderr=False)


@pytest.fixture
def CONFIG_JSON():
    p = TESTS_DIR / "pcaps" / "OS-0-128_v3.0.1_1024x10.json"
    if not p.exists():
        pytest.skip(f"Test fixture not found: {p}")
    return str(p)


@pytest.fixture
def SENSOR_HOSTNAME2():
    hostname = os.environ.get("SENSOR_HOSTNAME2")
    if not hostname:
        pytest.skip("SENSOR_HOSTNAME2 not set")
    return hostname


@pytest.fixture
def SAMPLE_DATA_OSF_PATH():
    p = TESTS_DIR / "osfs" / "single_scan_016.osf"
    if not p.exists():
        pytest.skip(f"Test fixture not found: {p}")
    return str(p)


@pytest.fixture
def MULTI_SENSOR_OSF_PATH():
    p = TESTS_DIR / "osfs" / "pose_delta_1_128.osf"
    if not p.exists():
        pytest.skip(f"Test fixture not found: {p}")
    return str(p)


@pytest.fixture
def PERCEPTION_OSF_PATH():
    p = TESTS_DIR / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"
    if not p.exists():
        pytest.skip(f"Test fixture not found: {p}")
    return str(p)


@pytest.fixture
def SAMPLE_DATA_PCAP_PATH():
    p = TESTS_DIR / "pcaps" / "OS-0-128-U1_v2.3.0_1024x10.pcap"
    if not p.exists():
        pytest.skip(f"Test fixture not found: {p}")
    return str(p)


@pytest.fixture
def SAMPLE_DATA_JSON_PATH():
    p = TESTS_DIR / "pcaps" / "OS-0-128-U1_v2.3.0_1024x10.json"
    if not p.exists():
        pytest.skip(f"Test fixture not found: {p}")
    return str(p)


@pytest.fixture
def LOOP_OSF_PATH():
    test_data_dir = os.environ.get("TEST_DATA_DIR")
    if not test_data_dir:
        pytest.skip("TEST_DATA_DIR not set")
    p = Path(test_data_dir) / "mapping" / "loop.osf"
    if not p.exists():
        pytest.skip(f"Test fixture not found: {p}")
    return str(p)


# ===========================================================================
# sensor_config
# ===========================================================================

def test_discover(runner):
    command = """
# [doc-stag-cli-discover]
ouster-cli discover
# [doc-etag-cli-discover]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args + ["--timeout", "1"])
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_auto_config(runner, SENSOR_HOSTNAME):
    command = f"""
# [doc-stag-cli-auto-config]
ouster-cli source {SENSOR_HOSTNAME} config
# [doc-etag-cli-auto-config]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_config_from_file(runner, SENSOR_HOSTNAME, CONFIG_JSON):
    command = f"""
# [doc-stag-cli-config-file]
ouster-cli source {SENSOR_HOSTNAME} config -c {CONFIG_JSON}
# [doc-etag-cli-config-file]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code in (0, 1), f"Output:\n{result.output}"


def test_custom_config(runner, SENSOR_HOSTNAME):
    command = f"""
# [doc-stag-cli-custom-config]
ouster-cli source {SENSOR_HOSTNAME} config lidar_mode 1024x10 udp_port_lidar 29847
# [doc-etag-cli-custom-config]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_custom_config_standby(runner, SENSOR_HOSTNAME):
    command = f"""
# [doc-stag-cli-custom-config-2]
ouster-cli source {SENSOR_HOSTNAME} config -s \
    lidar_mode 512x10 \
    azimuth_window "[90000, 270000]"
# [doc-etag-cli-custom-config-2]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_get_config_file(runner, SENSOR_HOSTNAME, tmp_path):
    command = f"""
# [doc-stag-cli-getconfig-file]
ouster-cli source {SENSOR_HOSTNAME} config -d > original_config.json
# [doc-etag-cli-getconfig-file]
    """
    args, redirect = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"
    assert result.output.strip(), "Expected config JSON output"
    if redirect:
        output_path = tmp_path / Path(redirect).name
        output_path.write_text(result.output)
        assert output_path.exists()


def test_get_config(runner, SENSOR_HOSTNAME):
    command = f"""
# [doc-stag-cli-get-config]
ouster-cli source {SENSOR_HOSTNAME} config -d
# [doc-etag-cli-get-config]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"
    assert result.output.strip(), "Expected config JSON in stdout"


def test_get_metadata(runner, SENSOR_HOSTNAME, tmp_path):
    VALID_INPUT_SOURCE = SENSOR_HOSTNAME
    command = f"""
# [doc-stag-cli-get-metadata]
ouster-cli source {VALID_INPUT_SOURCE} sensor_info > {SENSOR_HOSTNAME}.json
# [doc-etag-cli-get-metadata]
    """
    args, redirect = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"
    assert result.output.strip(), "Expected sensor_info JSON in stdout"
    if redirect:
        output_path = tmp_path / Path(redirect).name
        output_path.write_text(result.output)
        assert output_path.exists()


# ===========================================================================
# consumption
# ===========================================================================

def test_source_live(SENSOR_HOSTNAME):
    run_for_sec = 50.0
    command = f"""
# [doc-stag-cli-source-live]
ouster-cli source {SENSOR_HOSTNAME} stats --verbose
# [doc-etag-cli-source-live]
    """
    args, _ = command_to_args(command)
    out = _run_ouster_cli_subprocess_timeout(args, run_for_sec)
    assert "frames/sec" in out


@pytest.mark.interactive
def test_source_live_viz(runner, SENSOR_HOSTNAME):
    command = f"""
# [doc-stag-cli-source-live-viz]
ouster-cli source -x -y --timeout 2 {SENSOR_HOSTNAME} viz
# [doc-etag-cli-source-live-viz]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_source_multi_host(runner, SENSOR_HOSTNAME, SENSOR_HOSTNAME2):
    command = f"""
# [doc-stag-cli-source-multi-host]
ouster-cli source {SENSOR_HOSTNAME},{SENSOR_HOSTNAME2} stats
# [doc-etag-cli-source-multi-host]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_source_glob(runner, SAMPLE_DATA_OSF_PATH):
    OSF_GLOB = str(Path(SAMPLE_DATA_OSF_PATH).parent / "OS-*.osf")
    command = f"""
# [doc-stag-cli-source-glob]
ouster-cli source --glob "{OSF_GLOB}" stats
# [doc-etag-cli-source-glob]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_source_pcap_info(runner, SAMPLE_DATA_PCAP_PATH):
    command = f"""
# [doc-stag-cli-source-pcap-info]
ouster-cli source {SAMPLE_DATA_PCAP_PATH} info
# [doc-etag-cli-source-pcap-info]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_source_pcap_meta(runner, SAMPLE_DATA_PCAP_PATH, SAMPLE_DATA_JSON_PATH):
    command = f"""
# [doc-stag-cli-source-pcap-meta]
ouster-cli source --meta {SAMPLE_DATA_JSON_PATH} {SAMPLE_DATA_PCAP_PATH} info
# [doc-etag-cli-source-pcap-meta]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_source_single_sensor(runner, SAMPLE_DATA_PCAP_PATH):
    command = f"""
# [doc-stag-cli-source-single-sensor]
ouster-cli source --sensor-idx 0 {SAMPLE_DATA_PCAP_PATH} stats
# [doc-etag-cli-source-single-sensor]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_source_fields(runner, SAMPLE_DATA_OSF_PATH):
    command = f"""
# [doc-stag-cli-source-fields]
ouster-cli source --fields RANGE,REFLECTIVITY --filter {SAMPLE_DATA_OSF_PATH} stats
# [doc-etag-cli-source-fields]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_source_studio_viz(runner):
    command = """
# [doc-stag-cli-source-studio]
ouster-cli source "https://studio.ouster.com/share/V96PNUOMAATVXPPB" viz
# [doc-etag-cli-source-studio]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


# ===========================================================================
# playback
# ===========================================================================

def test_record_pcap_raw(runner, SENSOR_HOSTNAME, tmp_path):
    PCAP_OUTPUT_FILENAME = str(tmp_path / "recording")
    command = f"""
# [doc-stag-cli-record-pcap-raw]
ouster-cli source {SENSOR_HOSTNAME} save_raw --duration 5 {PCAP_OUTPUT_FILENAME}.pcap
# [doc-etag-cli-record-pcap-raw]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"
    assert (tmp_path / "recording.pcap").is_file(), f"expected {tmp_path / 'recording.pcap'}"


def test_record_bag(runner, SENSOR_HOSTNAME, tmp_path):
    BAG_OUTPUT_FILENAME = str(tmp_path / "recording")
    command = f"""
# [doc-stag-cli-record-bag]
ouster-cli source {SENSOR_HOSTNAME} save_raw --duration 5 {BAG_OUTPUT_FILENAME}.bag
# [doc-etag-cli-record-bag]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"
    assert (tmp_path / "recording.bag").is_file(), f"expected {tmp_path / 'recording.bag'}"


def test_record_pcap(SENSOR_HOSTNAME, tmp_path):
    run_for_sec = 10.0
    PCAP_OUTPUT_FILENAME = str(tmp_path / "recording")
    command = f"""
# [doc-stag-cli-record-pcap]
ouster-cli source {SENSOR_HOSTNAME} save {PCAP_OUTPUT_FILENAME}.pcap
# [doc-etag-cli-record-pcap]
    """
    args, _ = command_to_args(command)
    _run_ouster_cli_subprocess_timeout(args, run_for_sec)
    assert (tmp_path / "recording.pcap").is_file(), f"expected PCAP at {tmp_path / 'recording.pcap'}"


def test_record_osf(SENSOR_HOSTNAME, tmp_path):
    run_for_sec = 10.0
    OSF_OUTPUT_FILENAME = str(tmp_path / "recording")
    command = f"""
# [doc-stag-cli-record-osf]
ouster-cli source {SENSOR_HOSTNAME} save --overwrite {OSF_OUTPUT_FILENAME}.osf
# [doc-etag-cli-record-osf]
    """
    args, _ = command_to_args(command)
    _run_ouster_cli_subprocess_timeout(args, run_for_sec)
    assert (tmp_path / "recording.osf").is_file(), f"expected OSF at {tmp_path / 'recording.osf'}"


def test_record_csv(runner, SAMPLE_DATA_OSF_PATH, tmp_path):
    CSV_OUTPUT_DIR = str(tmp_path / "csv_output")
    VALID_INPUT_SOURCE = SAMPLE_DATA_OSF_PATH
    command = f"""
# [doc-stag-cli-record-csv]
ouster-cli source {VALID_INPUT_SOURCE} save --dir {CSV_OUTPUT_DIR} ".csv"
# [doc-etag-cli-record-csv]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_record_png(runner, SAMPLE_DATA_OSF_PATH, tmp_path):
    PNG_OUTPUT_DIR = str(tmp_path / "png_output")
    Path(PNG_OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
    VALID_INPUT_SOURCE = SAMPLE_DATA_OSF_PATH
    command = f"""
# [doc-stag-cli-record-png]
ouster-cli source {VALID_INPUT_SOURCE} save --dir {PNG_OUTPUT_DIR} ".png"
# [doc-etag-cli-record-png]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_sensor_replay_help(runner, SAMPLE_DATA_OSF_PATH):
    command = f"""
# [doc-stag-cli-sensor-replay-help]
ouster-cli source {SAMPLE_DATA_OSF_PATH} sensor_replay --help
# [doc-etag-cli-sensor-replay-help]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_sensor_replay(SAMPLE_DATA_OSF_PATH):
    REPLAY_SOURCE = SAMPLE_DATA_OSF_PATH
    command = f"""
# [doc-stag-cli-sensor-replay]
ouster-cli source {REPLAY_SOURCE} sensor_replay --http-port 8080
# [doc-etag-cli-sensor-replay]
    """
    args, _ = command_to_args(command)
    out = _run_ouster_cli_subprocess_timeout(args, run_for_sec=5)
    assert "Running on http://127.0.0.1:8080" in out, f"sensor_replay did not start:\n{out}"


@pytest.mark.interactive
def test_viz_osf(runner, SAMPLE_DATA_OSF_PATH):
    command = f"""
# [doc-stag-cli-viz-osf]
ouster-cli source {SAMPLE_DATA_OSF_PATH} viz -r 1.5 -e loop
# [doc-etag-cli-viz-osf]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_viz_sensor(runner, SENSOR_HOSTNAME):
    command = f"""
# [doc-stag-cli-viz-sensor]
ouster-cli source {SENSOR_HOSTNAME} viz
# [doc-etag-cli-viz-sensor]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_viz_pcap(runner, SAMPLE_DATA_JSON_PATH, SAMPLE_DATA_PCAP_PATH):
    command = f"""
# [doc-stag-cli-viz-pcap]
ouster-cli source --meta {SAMPLE_DATA_JSON_PATH} {SAMPLE_DATA_PCAP_PATH} viz
# [doc-etag-cli-viz-pcap]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_viz_extrinsics(runner, SENSOR_HOSTNAME):
    command = f"""
# [doc-stag-cli-viz-extrinsics]
ouster-cli source --extrinsics "-1 0 0 0 0 1 0 0 0 0 -1 0 0 0 0 1" {SENSOR_HOSTNAME} viz
# [doc-etag-cli-viz-extrinsics]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


# ===========================================================================
# viz (getting-started)
# ===========================================================================

@pytest.mark.interactive
def test_viz_from_pcap(runner, SAMPLE_DATA_JSON_PATH, SAMPLE_DATA_PCAP_PATH):
    command = f"""
# [doc-stag-viz-from-pcap]
ouster-cli source --meta {SAMPLE_DATA_JSON_PATH} {SAMPLE_DATA_PCAP_PATH} viz
# [doc-etag-viz-from-pcap]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_viz_from_sensor(runner, SENSOR_HOSTNAME):
    command = f"""
# [doc-stag-viz-from-sensor]
ouster-cli source {SENSOR_HOSTNAME} viz
# [doc-etag-viz-from-sensor]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_viz_with_extrinsics(runner, SENSOR_HOSTNAME):
    command = f"""
# [doc-stag-viz-with-extrinsics]
ouster-cli source --extrinsics "-1 0 0 0 0 1 0 0 0 0 -1 0 0 0 0 1" {SENSOR_HOSTNAME} viz
# [doc-etag-viz-with-extrinsics]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_viz_help(runner, SAMPLE_DATA_PCAP_PATH):
    command = f"""
# [doc-stag-viz-help]
ouster-cli source {SAMPLE_DATA_PCAP_PATH} viz --help
# [doc-etag-viz-help]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_viz_subframes(runner, SAMPLE_DATA_PCAP_PATH):
    command = f"""
# [doc-stag-viz-subframes]
ouster-cli source {SAMPLE_DATA_PCAP_PATH} viz --subframes 10
# [doc-etag-viz-subframes]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


# ===========================================================================
# processing
# ===========================================================================


def test_slice_osf(runner, SAMPLE_DATA_OSF_PATH, tmp_path):
    SLICED_OUTPUT = str(tmp_path / "sliced")
    command = f"""
# [doc-stag-cli-slice-osf]
ouster-cli source {SAMPLE_DATA_OSF_PATH} slice 0:100 save --overwrite {SLICED_OUTPUT}.osf
# [doc-etag-cli-slice-osf]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_clip_help(runner, SAMPLE_DATA_OSF_PATH):
    HOSTNAME_OR_VALID_IN_FILENAME = SAMPLE_DATA_OSF_PATH
    command = f"""
# [doc-stag-clip-help]
ouster-cli source {HOSTNAME_OR_VALID_IN_FILENAME} clip --help
# [doc-etag-clip-help]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


# Filter syntax reference for documentation (not a runnable command)
_FILTER_SYNTAX = """
# [doc-stag-filter]
# ouster-cli source {HOSTNAME_OR_VALID_IN_FILENAME} filter [OPTIONS] AXIS_FIELD INDICES
# [doc-etag-filter]
"""


@pytest.mark.interactive
def test_cli_clip(runner, SAMPLE_DATA_OSF_PATH, tmp_path):
    HOSTNAME_OR_VALID_IN_FILENAME = SAMPLE_DATA_OSF_PATH
    CLIP_OUTPUT = str(tmp_path / "clipped")
    command = f"""
# [doc-stag-cli-clip]
ouster-cli source {HOSTNAME_OR_VALID_IN_FILENAME} clip RANGE,RANGE2 20m:50m viz save {CLIP_OUTPUT}.pcap
# [doc-etag-cli-clip]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_clip_viz(runner, SAMPLE_DATA_OSF_PATH, tmp_path):
    HOSTNAME_OR_VALID_IN_FILENAME = SAMPLE_DATA_OSF_PATH
    CLIP_OUTPUT = str(tmp_path / "clipped_2")
    command = f"""
# [doc-stag-clip-viz]
ouster-cli source {HOSTNAME_OR_VALID_IN_FILENAME} viz clip RANGE,RANGE2 20m:50m save {CLIP_OUTPUT}.pcap
# [doc-etag-clip-viz]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_clip_slam_viz(runner, SAMPLE_DATA_OSF_PATH, tmp_path):
    HOSTNAME_OR_VALID_IN_FILENAME = SAMPLE_DATA_OSF_PATH
    CLIP_OUTPUT_1 = str(tmp_path / "clipped_3")
    CLIP_OUTPUT_2 = str(tmp_path / "clipped_4")
    command = f"""
# [doc-stag-clip-slam-viz]
ouster-cli source {HOSTNAME_OR_VALID_IN_FILENAME} slam clip RANGE,RANGE2 20m:50m viz save {CLIP_OUTPUT_1}.ply
ouster-cli source {HOSTNAME_OR_VALID_IN_FILENAME} slam --min-range 10 --max-range 100 clip RANGE,RANGE2 20m:50m viz save {CLIP_OUTPUT_2}.ply
# [doc-etag-clip-slam-viz]
    """  # noqa: E501
    for line in command.strip().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        args, _ = command_to_args(line)
        result = runner.invoke(core.cli, args)
        assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_filter_rfl(runner, SAMPLE_DATA_OSF_PATH):
    HOSTNAME_OR_VALID_IN_FILENAME = SAMPLE_DATA_OSF_PATH
    command = f"""
# [doc-stag-filter-rfl]
ouster-cli source {HOSTNAME_OR_VALID_IN_FILENAME} filter REFLECTIVITY :50 viz
# [doc-etag-filter-rfl]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_filter_z(runner, SAMPLE_DATA_OSF_PATH):
    HOSTNAME_OR_VALID_IN_FILENAME = SAMPLE_DATA_OSF_PATH
    command = f"""
# [doc-stag-filter-z]
ouster-cli source {HOSTNAME_OR_VALID_IN_FILENAME} filter Z :-1m filter Z 1m: viz
# [doc-etag-filter-z]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_filter_coord(runner, SAMPLE_DATA_OSF_PATH):
    HOSTNAME_OR_VALID_IN_FILENAME = SAMPLE_DATA_OSF_PATH
    command = f"""
# [doc-stag-filter-coord]
ouster-cli source {HOSTNAME_OR_VALID_IN_FILENAME} filter V 256:768 viz
# [doc-etag-filter-coord]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_filter_opts(runner, SAMPLE_DATA_OSF_PATH):
    HOSTNAME_OR_VALID_IN_FILENAME = SAMPLE_DATA_OSF_PATH
    command = f"""
# [doc-stag-filter-opts]
ouster-cli source {HOSTNAME_OR_VALID_IN_FILENAME} filter --filtered-fields REFLECTIVITY,NEAR_IR --invalid-value 100 U 30:40 viz
# [doc-etag-filter-opts]
    """  # noqa: E501
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_mask(runner, SAMPLE_DATA_OSF_PATH, tmp_path):
    from PIL import Image
    HOSTNAME_OR_VALID_IN_FILENAME = SAMPLE_DATA_OSF_PATH
    MASK_IMAGE = str(tmp_path / "mask.png")
    Image.new('L', (1024, 128), 255).save(MASK_IMAGE)
    command = f"""
# [doc-stag-mask]
ouster-cli source {HOSTNAME_OR_VALID_IN_FILENAME} mask --fields RANGE {MASK_IMAGE} viz
# [doc-etag-mask]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_reduce(runner, SAMPLE_DATA_OSF_PATH):
    HOSTNAME_OR_VALID_IN_FILENAME = SAMPLE_DATA_OSF_PATH
    command = f"""
# [doc-stag-reduce]
ouster-cli source {HOSTNAME_OR_VALID_IN_FILENAME} reduce 32 viz
# [doc-etag-reduce]
    """
    args, _ = command_to_args(command)
    with patch("sys.argv", ["ouster-cli"] + args):
        result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_reduce_save(runner, SAMPLE_DATA_OSF_PATH, tmp_path):
    HOSTNAME_OR_VALID_IN_FILENAME = SAMPLE_DATA_OSF_PATH
    REDUCED_OUTPUT = str(tmp_path / "reduced")
    command = f"""
ouster-cli source {HOSTNAME_OR_VALID_IN_FILENAME} reduce 32 save --overwrite {REDUCED_OUTPUT}.pcap
    """
    args, _ = command_to_args(command)
    with patch("sys.argv", ["ouster-cli"] + args):
        result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


# ===========================================================================
# normals
# ===========================================================================

@pytest.mark.interactive
def test_cli_normals_viz(runner, SAMPLE_DATA_OSF_PATH):
    command = """
# [doc-stag-cli-normals-viz]
ouster-cli source {OSF_PATH} normals viz
# [doc-etag-cli-normals-viz]
    """
    args, _ = command_to_args(command)
    args[args.index("{OSF_PATH}")] = SAMPLE_DATA_OSF_PATH
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_cli_normals_sensor_coord_viz(runner, SAMPLE_DATA_OSF_PATH):
    command = """
# [doc-stag-cli-normals-sensor-coord-viz]
ouster-cli source {OSF_PATH} normals --sensor-coord viz
# [doc-etag-cli-normals-sensor-coord-viz]
    """
    args, _ = command_to_args(command)
    args[args.index("{OSF_PATH}")] = SAMPLE_DATA_OSF_PATH
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.parametrize("sensor_coord", [False, True])
def test_cli_normals_save(
        runner, SAMPLE_DATA_OSF_PATH, tmp_path, sensor_coord):
    output = tmp_path / (
        "normals-sensor-coord.osf" if sensor_coord else "normals.osf")
    args = ["source", SAMPLE_DATA_OSF_PATH, "normals"]
    if sensor_coord:
        args.append("--sensor-coord")
    args.extend(["save", "--overwrite", str(output)])

    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"

    with sdk_open_source(str(output)) as frame_set_source:
        frame = next(iter(frame_set_source))[0]
        assert frame is not None
        assert frame.has_field(sdk_core.ChanField.NORMALS)


# ===========================================================================
# ground_seg
# ===========================================================================

@pytest.mark.interactive
def test_cli_ground_viz(runner, SAMPLE_DATA_OSF_PATH):
    command = """
# [doc-stag-cli-ground-viz]
ouster-cli source {OSF_PATH} ground viz
# [doc-etag-cli-ground-viz]
    """
    args, _ = command_to_args(command)
    args[args.index("{OSF_PATH}")] = SAMPLE_DATA_OSF_PATH
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_cli_plumb_ground_viz(runner, SAMPLE_DATA_PCAP_PATH):
    command = """
# [doc-stag-cli-plumb-ground-viz]
ouster-cli source {SOURCE_PATH} plumb ground viz
# [doc-etag-cli-plumb-ground-viz]
    """
    args, _ = command_to_args(command)
    args[args.index("{SOURCE_PATH}")] = SAMPLE_DATA_PCAP_PATH
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_cli_plumb_slam_ground_viz(runner, SAMPLE_DATA_PCAP_PATH):
    command = """
# [doc-stag-cli-plumb-slam-ground-viz]
ouster-cli source {SOURCE_PATH} plumb slam ground viz
# [doc-etag-cli-plumb-slam-ground-viz]
    """
    args, _ = command_to_args(command)
    args[args.index("{SOURCE_PATH}")] = SAMPLE_DATA_PCAP_PATH
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_cli_ground_save(runner, SAMPLE_DATA_OSF_PATH, tmp_path):
    output = tmp_path / "ground.osf"
    args = [
        "source", SAMPLE_DATA_OSF_PATH, "ground",
        "save", "--overwrite", str(output),
    ]

    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"

    with sdk_open_source(str(output)) as frame_set_source:
        frame = next(iter(frame_set_source))[0]
        assert frame is not None
        assert frame.has_field(sdk_core.ChanField.GROUND)


def test_cli_plumb_ground_save(runner, SAMPLE_DATA_PCAP_PATH, tmp_path):
    output = tmp_path / "plumb-ground.osf"
    args = [
        "source", SAMPLE_DATA_PCAP_PATH, "plumb", "ground",
        "save", "--overwrite", str(output),
    ]

    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"

    with sdk_open_source(str(output)) as frame_set_source:
        frame = next(iter(frame_set_source))[0]
        assert frame is not None
        assert frame.has_field(sdk_core.ChanField.GROUND)


def test_cli_plumb_slam_ground_save(
        runner, SAMPLE_DATA_PCAP_PATH, tmp_path):
    output = tmp_path / "plumb-slam-ground.osf"
    args = [
        "source", SAMPLE_DATA_PCAP_PATH, "plumb", "slam", "ground",
        "save", "--overwrite", str(output),
    ]

    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"

    with sdk_open_source(str(output)) as frame_set_source:
        frame = next(iter(frame_set_source))[0]
        assert frame is not None
        assert frame.has_field(sdk_core.ChanField.GROUND)


# ===========================================================================
# mapping
# ===========================================================================

def test_slam_help(runner, SAMPLE_DATA_OSF_PATH):
    command = f"""
# [doc-slam-help-begin]
ouster-cli source {SAMPLE_DATA_OSF_PATH} slam --help
# [doc-slam-help-end]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_slam_save_help(runner, SAMPLE_DATA_OSF_PATH):
    command = f"""
# [doc-slam-save-help-begin]
ouster-cli source {SAMPLE_DATA_OSF_PATH} slam save --help
# [doc-slam-save-help-end]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_slam_viz_help(runner, SAMPLE_DATA_OSF_PATH):
    command = f"""
# [doc-slam-viz-help-begin]
ouster-cli source {SAMPLE_DATA_OSF_PATH} slam viz --help
# [doc-slam-viz-help-end]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_slam_save(runner, SAMPLE_DATA_OSF_PATH, tmp_path):
    SLAM_OUTPUT = str(tmp_path / "slam")
    command = f"""
# [doc-slam-save-begin]
ouster-cli source {SAMPLE_DATA_OSF_PATH} slam save --overwrite {SLAM_OUTPUT}.pcap
# [doc-slam-save-end]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_slam_save_ply(runner, SAMPLE_DATA_OSF_PATH, tmp_path):
    PLY_OUTPUT = str(tmp_path / "output")
    command = f"""
# [doc-slam-saveply-begin]
ouster-cli source {SAMPLE_DATA_OSF_PATH} slam save --overwrite {PLY_OUTPUT}.ply
# [doc-slam-saveply-end]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_clip_save_ply(runner, SAMPLE_DATA_OSF_PATH, tmp_path):
    CLIP_OUTPUT = str(tmp_path / "clipped_output")
    command = f"""
# [doc-clipply-begin]
ouster-cli source {SAMPLE_DATA_OSF_PATH} clip RANGE,RANGE2 20m:80m save --overwrite {CLIP_OUTPUT}.ply
# [doc-clipply-end]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_slam_viz(runner, SAMPLE_DATA_OSF_PATH, tmp_path):
    SOURCE_URL = SAMPLE_DATA_OSF_PATH
    SAMPLE_OUTPUT = str(tmp_path / "sample")
    command = f"""
# [doc-slamviz-begin]
ouster-cli source {SOURCE_URL} slam viz -e exit save {SAMPLE_OUTPUT}.osf
# [doc-slamviz-end]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_localize(runner, SOURCE, MAP):
    command = f"""
# [doc-localize-begin]
ouster-cli source {SOURCE} localize {MAP} viz
# [doc-localize-end]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_mapping_localize_initpose(runner, SOURCE, MAP):
    PX, PY, PZ = 1.5, 2.0, 0.0
    R, P, Y = 0.0, 0.0, 45.0
    command = f"""
# [doc-initpose-begin]
ouster-cli source --initial-pose {PX},{PY},{PZ},{R},{P},{Y} {SOURCE} localize {MAP} viz
# [doc-initpose-end]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_mapping_localize_flatmap(runner, SOURCE, MAP):
    command = f"""
# [doc-flatmap-begin]
ouster-cli source {SOURCE} localize {MAP} viz --global-map-flatten False
# [doc-flatmap-end]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_slam_to_osf_to_map_ply(runner, SOURCE, tmp_path):
    OSF_OUTPUT = str(tmp_path / "output")
    MAP_OUTPUT = str(tmp_path / "map")

    slam_command = f"""
# [doc-slam-to-osf-begin]
ouster-cli source {SOURCE} slam save --ts lidar --overwrite {OSF_OUTPUT}.osf
# [doc-slam-to-osf-end]
    """
    args, _ = command_to_args(slam_command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"slam save output:\n{result.output}"

    ply_command = f"""
# [doc-osf-to-map-ply-begin]
ouster-cli source {OSF_OUTPUT}.osf save {MAP_OUTPUT}.ply
# [doc-osf-to-map-ply-end]
    """
    args, _ = command_to_args(ply_command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"save ply output:\n{result.output}"


# ===========================================================================
# point_cloud_alignment
# ===========================================================================

@pytest.mark.interactive
def test_cli_align_viz(runner, MULTI_SENSOR_OSF_PATH):
    command = f"""
# [doc-stag-cli-align-viz]
ouster-cli source {MULTI_SENSOR_OSF_PATH} align viz
# [doc-etag-cli-align-viz]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_cli_plumb_align_viz(runner, SENSOR_HOSTNAME, SENSOR_HOSTNAME2):
    command = f"""
# [doc-stag-cli-plumb-align-viz]
ouster-cli source {SENSOR_HOSTNAME},{SENSOR_HOSTNAME2} plumb align viz
# [doc-etag-cli-plumb-align-viz]
    """
    args, _ = command_to_args(command)
    args = [args[0], "-x", "-y", "--timeout", "2", *args[1:]]
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


def test_cli_align_save(runner, MULTI_SENSOR_OSF_PATH, tmp_path):
    command = f"""
# [doc-stag-cli-align-save]
ouster-cli source {MULTI_SENSOR_OSF_PATH} align save aligned.osf
# [doc-etag-cli-align-save]
    """
    args, _ = command_to_args(command)
    args[args.index("aligned.osf")] = str(tmp_path / "aligned.osf")
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"
    assert (tmp_path / "aligned.osf").is_file(), (
        f"expected {(tmp_path / 'aligned.osf')}"
    )


# ===========================================================================
# pose_optimizer
# ===========================================================================

@pytest.fixture
def CONSTRAINTS_JSON():
    p = (REPO_ROOT / "docs" / "features" / "pose_optimizer"
         / "_snippets" / "cli" / "pose_optimizer_cli_constraints.json")
    if not p.exists():
        pytest.skip(f"Test fixture not found: {p}")
    return str(p)


def test_po_help(runner, SAMPLE_DATA_OSF_PATH):
    command = """
# [doc-po-cli-help-begin]
ouster-cli source loop.osf pose_optimize --help
# [doc-po-cli-help-end]
    """
    args, _ = command_to_args(command)
    args[args.index("loop.osf")] = SAMPLE_DATA_OSF_PATH
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.slow
def test_po_run(runner, LOOP_OSF_PATH, CONSTRAINTS_JSON, tmp_path):
    OPTIMIZED_OUTPUT = str(tmp_path / "optimized_loop")
    command = """
# [doc-po-cli-run-begin]
ouster-cli source loop.osf pose_optimize --config constraints.json optimized_loop.osf
# [doc-po-cli-run-end]
    """
    args, _ = command_to_args(command)
    args[args.index("loop.osf")] = LOOP_OSF_PATH
    args[args.index("constraints.json")] = CONSTRAINTS_JSON
    args[args.index("optimized_loop.osf")] = f"{OPTIMIZED_OUTPUT}.osf"
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_po_viz(runner, LOOP_OSF_PATH, CONSTRAINTS_JSON, tmp_path):
    OPTIMIZED_OUTPUT = str(tmp_path / "optimized_loop")
    command = """
# [doc-po-cli-viz-begin]
ouster-cli source loop.osf pose_optimize --config constraints.json --viz optimized_loop.osf
# [doc-po-cli-viz-end]
    """
    args, _ = command_to_args(command)
    args[args.index("loop.osf")] = LOOP_OSF_PATH
    args[args.index("constraints.json")] = CONSTRAINTS_JSON
    args[args.index("optimized_loop.osf")] = f"{OPTIMIZED_OUTPUT}.osf"
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_po_auto_gps(runner, LOOP_OSF_PATH, CONSTRAINTS_JSON, tmp_path):
    OPTIMIZED_OUTPUT = str(tmp_path / "optimized_loop")
    subs = {"loop.osf": LOOP_OSF_PATH, "constraints.json": CONSTRAINTS_JSON,
            "optimized_loop.osf": f"{OPTIMIZED_OUTPUT}.osf"}
    command = """
# [doc-po-cli-auto-gps-begin]
ouster-cli source loop.osf pose_optimize --auto-gps --viz optimized_loop.osf
ouster-cli source loop.osf pose_optimize --auto-gps --config constraints.json --viz optimized_loop.osf
# [doc-po-cli-auto-gps-end]
    """
    run_multiline_command(command, subs, runner, core.cli)


@pytest.mark.interactive
def test_po_auto_loop(runner, LOOP_OSF_PATH, tmp_path):
    OPTIMIZED_OUTPUT = str(tmp_path / "optimized_loop")
    subs = {"loop.osf": LOOP_OSF_PATH,
            "optimized_loop.osf": f"{OPTIMIZED_OUTPUT}.osf"}
    command = """
# [doc-po-cli-auto-loop-begin]
ouster-cli source loop.osf pose_optimize --auto-loop optimized_loop.osf
ouster-cli source loop.osf pose_optimize --auto-loop --viz optimized_loop.osf
# [doc-po-cli-auto-loop-end]
    """
    run_multiline_command(command, subs, runner, core.cli)


@pytest.mark.interactive
def test_po_auto_gps_loop(runner, LOOP_OSF_PATH, tmp_path):
    OPTIMIZED_OUTPUT = str(tmp_path / "optimized_loop")
    subs = {"loop.osf": LOOP_OSF_PATH,
            "optimized_loop.osf": f"{OPTIMIZED_OUTPUT}.osf"}
    command = """
# [doc-po-cli-auto-gps-loop-begin]
ouster-cli source loop.osf pose_optimize --auto-gps --auto-loop optimized_loop.osf
ouster-cli source loop.osf pose_optimize --auto-gps --auto-loop --viz optimized_loop.osf
# [doc-po-cli-auto-gps-loop-end]
    """
    run_multiline_command(command, subs, runner, core.cli)


@pytest.mark.interactive
def test_po_map(runner, LOOP_OSF_PATH):
    command = """
# [doc-po-cli-map-begin]
ouster-cli source loop.osf viz --map
# [doc-po-cli-map-end]
    """
    args, _ = command_to_args(command)
    args[args.index("loop.osf")] = LOOP_OSF_PATH
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


# ===========================================================================
# zone_monitor
# ===========================================================================

@pytest.fixture
def ZM_CONFIG_ZIP():
    p = TESTS_DIR / "zone_monitor" / "single_scan_016_zone_config.zip"
    if not p.exists():
        pytest.skip(f"Test fixture not found: {p}")
    return str(p)


def test_zm_emulate(runner, SAMPLE_DATA_OSF_PATH, ZM_CONFIG_ZIP, tmp_path):
    SOURCE = SAMPLE_DATA_OSF_PATH
    CONFIG_ZIP = ZM_CONFIG_ZIP
    OUTPUT_FILE = str(tmp_path / "test")
    command = f"""
# [doc-stag-zm-emulate]
ouster-cli source {SOURCE} emulate_zones -c {CONFIG_ZIP} save {OUTPUT_FILE}.osf
# [doc-etag-zm-emulate]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_zm_emulate_viz(runner, SAMPLE_DATA_OSF_PATH, ZM_CONFIG_ZIP):
    SOURCE = SAMPLE_DATA_OSF_PATH
    CONFIG_ZIP = ZM_CONFIG_ZIP
    command = f"""
# [doc-stag-zm-emulate-viz]
ouster-cli source {SOURCE} emulate_zones -c {CONFIG_ZIP} viz
# [doc-etag-zm-emulate-viz]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_zm_viz(runner, SAMPLE_DATA_OSF_PATH):
    SOURCE = SAMPLE_DATA_OSF_PATH
    command = f"""
# [doc-stag-zm-viz]
ouster-cli source {SOURCE} viz
# [doc-etag-zm-viz]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


# ===========================================================================
# localization
# ===========================================================================

LOCALIZATION_OSF = TESTS_DIR / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"
LOCALIZATION_MAP = TESTS_DIR / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3_map-000.ply"


@pytest.fixture
def SOURCE():
    """OSF file used as the lidar source for localization doc-snippet tests."""
    if not LOCALIZATION_OSF.exists():
        pytest.skip(f"Localization test OSF not found: {LOCALIZATION_OSF}")
    return str(LOCALIZATION_OSF)


@pytest.fixture
def MAP():
    """Path to a pre-built PLY/PCD localization map.
    Tests are skipped if neither path resolves to an existing file.
    """
    p = Path(os.environ.get("TEST_MAP", str(LOCALIZATION_MAP)))
    if not p.exists():
        pytest.skip(f"Localization map not found: {p} (set TEST_MAP to override)")
    return str(p)


def test_localize_help(runner, SOURCE):
    command = f"""
# [doc-stag-localize-help]
ouster-cli source {SOURCE} localize --help
# [doc-etag-localize-help]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, \
        f"Output:\n{result.output}\nStderr:\n{result.stderr if hasattr(result, 'stderr') else 'N/A'}"


@pytest.mark.interactive
def test_localize_basic(runner, SOURCE, MAP):
    command = f"""
# [doc-stag-localize-basic]
ouster-cli source {SOURCE} localize {MAP} viz
# [doc-etag-localize-basic]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.fixture
def SENSOR_HOSTNAME():
    hostname = os.environ.get("SENSOR_HOSTNAME")
    if not hostname:
        pytest.skip("SENSOR_HOSTNAME not set")
    return hostname


def test_localize_save(runner, SOURCE, MAP, tmp_path):
    pytest.skip(
        "Skipped: SIGABRT on Debug builds due to Eigen assertion "
        "ERROR: Assertion failed: ((!(RowsAtCompileTime!=Dynamic) || (rows==RowsAtCompileTime)) && "
        "(!(ColsAtCompileTime!=Dynamic) || (cols==ColsAtCompileTime)) && (!(RowsAtCompileTime==Dynamic && "
        "MaxRowsAtCompileTime!=Dynamic) || (rows<=MaxRowsAtCompileTime)) && "
        "(!(ColsAtCompileTime==Dynamic && MaxColsAtCompileTime!=Dynamic) || (cols<=MaxColsAtCompileTime)) "
        "&& rows>=0 && cols>=0 && \"Invalid sizes when resizing a matrix or array.\"), function resize, "
        "file PlainObjectBase.h, line 277."
    )
    OUTPUT_OSF = str(tmp_path / "output.osf")
    command = f"""
# [doc-stag-localize-save]
ouster-cli source {SOURCE} localize {MAP} save {OUTPUT_OSF}
# [doc-etag-localize-save]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, \
        f"Output:\n{result.output}\nStderr:\n{result.stderr if hasattr(result, 'stderr') else 'N/A'}"


@pytest.mark.interactive
def test_localize_initpose(runner, SOURCE, MAP):
    PX, PY, PZ = 1.5, 2.0, 0.0    # metres offset from map origin (X east, Y north)
    R, P, Y = 0.0, 0.0, 45.0       # roll, pitch, yaw in degrees
    command = f"""
# [doc-stag-localize-initpose]
ouster-cli source --initial-pose {PX},{PY},{PZ},{R},{P},{Y} {SOURCE} localize {MAP} viz
# [doc-etag-localize-initpose]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code in (0, 1), f"Output:\n{result.output}"


@pytest.mark.interactive
def test_localize_ranges(runner, SOURCE, MAP):
    command = f"""
# [doc-stag-localize-ranges]
ouster-cli source {SOURCE} localize {MAP} --min-range 0.5 --max-range 80.0 viz
# [doc-etag-localize-ranges]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_localize_voxel(runner, SOURCE, MAP):
    command = f"""
# [doc-stag-localize-voxel]
ouster-cli source {SOURCE} localize {MAP} --voxel-size 1.0 viz
# [doc-etag-localize-voxel]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_localize_deskew(runner, SOURCE, MAP):
    command = f"""
# [doc-stag-localize-deskew]
ouster-cli source {SOURCE} localize {MAP} --deskew-method constant_velocity viz
# [doc-etag-localize-deskew]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_localize_flatmap(runner, SOURCE, MAP):
    command = f"""
# [doc-stag-localize-flatmap]
ouster-cli source {SOURCE} localize {MAP} viz --global-map-flatten False
# [doc-etag-localize-flatmap]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


@pytest.mark.interactive
def test_localize_globalmap_opts(runner, SOURCE, MAP):
    command = f"""
# [doc-stag-localize-globalmap-opts]
ouster-cli source {SOURCE} localize {MAP} viz \
    --global-map-flatten True \
    --global-map-min-z -1.0 \
    --global-map-max-z 5.0 \
    --global-map-voxel-size 0.5 \
    --global-map-point-size 2.0
# [doc-etag-localize-globalmap-opts]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"


# ---------------------------------------------------------------------------
# Perception / detect
# ---------------------------------------------------------------------------

@pytest.mark.perception
def test_source_detect(runner, PERCEPTION_OSF_PATH, tmp_path):
    OSF_FILE = PERCEPTION_OSF_PATH
    OUTPUT_OSF_FILE = str(tmp_path / "output.osf")
    command = f"""
# [doc-stag-source-detect]
ouster-cli source {OSF_FILE} detect save {OUTPUT_OSF_FILE}
# [doc-etag-source-detect]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"
    assert "Saving OSF file" in result.output


@pytest.mark.interactive
def test_source_viz(runner, PERCEPTION_OSF_PATH):
    OSF_FILE = PERCEPTION_OSF_PATH
    command = f"""
# [doc-stag-source-viz]
ouster-cli source {OSF_FILE} viz
# [doc-etag-source-viz]
    """
    args, _ = command_to_args(command)
    result = runner.invoke(core.cli, args)
    assert result.exit_code == 0, f"Output:\n{result.output}"
