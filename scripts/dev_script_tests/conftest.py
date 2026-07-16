import os
import pytest
import sys
import inspect
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "dev_script_library")))

from context import ClickContext  # noqa E402


def spy_on_all_methods(obj):
    """
    Iterates over an object, finds all bound methods (that are not dunders),
    and wraps them in a MagicMock spy.
    """
    for name in dir(obj):
        # Skip magic methods (start/end with __) to avoid breaking internal python mechanics
        if name.startswith("__") and name.endswith("__"):
            continue

        attr = getattr(obj, name)

        # Check if it is a bound method
        if inspect.ismethod(attr):
            # Create a spy that runs the real code but records calls
            spy = MagicMock(wraps=attr)
            setattr(obj, name, spy)


@pytest.fixture
def mock_build_libs():
    """
    Mock the heavy external build libraries (CMake, RunCommand).
    This prevents actual subprocess calls.
    """
    libs = MagicMock()

    # Setup the RunCommand mock to return a mock runner instance
    runner_instance = MagicMock()
    libs.RunCommand.return_value = runner_instance

    # Setup default 'true' responses for tool checks so builds don't abort early
    libs.check_for_python_lib.return_value = True
    libs.check_for_tool.return_value = True

    return libs


@pytest.fixture
def mock_context(mock_build_libs, tmp_path):
    with patch("os.makedirs"), \
         patch("os.path.isdir", return_value=True), \
         patch("os.path.exists", return_value=True), \
         patch("subprocess.call", return_value=0):
        ctx = ClickContext(mock_build_libs)
        fake_root = str(tmp_path / "mock_root")
        ctx.sdk_dir = os.path.join(fake_root, "sdk")
        ctx._dev_persistent_dir = os.path.join(fake_root, ".osdkv2")
        build_opts = ctx.build_options
        spy_on_all_methods(ctx)
        spy_on_all_methods(build_opts)

        yield ctx


@pytest.fixture
def clang_build_analyzer_checker():
    def _clang_build_analyzer_checker(call_list, cmake_build_dir, expected_cap_file):
        found_start_call = False
        found_stop_call = False
        found_analyze_call = False
        for call_obj in call_list:
            args, _ = call_obj
            if "ClangBuildAnalyzer" in args:
                if "--stop" in args and cmake_build_dir in args and expected_cap_file in args:
                    found_stop_call = True
                if "--start" in args and cmake_build_dir in args:
                    found_start_call = True
                if "--analyze" in args and expected_cap_file in args:
                    found_analyze_call = True
        assert found_stop_call
        assert found_start_call
        assert found_analyze_call

    return _clang_build_analyzer_checker
