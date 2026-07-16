import os
import sys
import pytest
from unittest.mock import patch
from click.testing import CliRunner

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "dev_script_library")))

from dev_python import python_build  # noqa E402
from dev_python import python_test  # noqa E402


def test_build_python_editable_default(mock_context):
    """Test default build (Editable mode, Release, Manifest ON)."""
    runner = CliRunner()
    result = runner.invoke(python_build, [], obj=mock_context)

    assert result.exit_code == 0

    # Check Env Setup
    expected_env_subset = {
        "VCPKG_MANIFEST_MODE": "ON",
        "VCPKG_ROOT": mock_context.vcpkg_dir,
        "BUILD_TYPE": "Release"
    }

    run_mock = mock_context.build_libs.RunCommand.return_value
    call_args = run_mock.run_command.call_args
    assert call_args is not None
    _, kwargs = call_args
    passed_env = kwargs.get('env', {})

    for key, val in expected_env_subset.items():
        assert passed_env[key] == val

    # Verify pip command
    expected_args = [
        sys.executable, "-m", "pip", "install",
        os.path.join(mock_context.sdk_dir, "python")
    ]

    assert run_mock.run_command.call_args[0] == tuple(expected_args)


def test_build_python_wheel(mock_context):
    """Test building a wheel."""
    runner = CliRunner()
    result = runner.invoke(python_build, ['--output-type', 'wheel'], obj=mock_context)

    assert result.exit_code == 0

    run_mock = mock_context.build_libs.RunCommand.return_value

    # Verify wheel arguments
    expected_args = [
        sys.executable, "-m", "pip", "wheel",
        "--no-deps", "--wheel-dir", mock_context.sdk_artifact_dir,
        os.path.join(mock_context.sdk_dir, "python")
    ]
    assert run_mock.run_command.call_args[0] == tuple(expected_args)

    # Verify we printed output location
    mock_context.print_output_location.assert_called_with("Python SDK Wheel")


@patch("dev_python.platform.machine", return_value="ppc")
@patch("dev_python.platform.system", return_value="Darwin")
def test_build_python_wheel_macos_warns_on_unknown_arch(_mock_system, _mock_machine, mock_context):
    """Darwin wheel builds warn when platform.machine() is not arm64/x86_64."""
    runner = CliRunner()
    with pytest.warns(UserWarning, match="Cannot infer macOS wheel architecture"):
        result = runner.invoke(
            python_build,
            ['--output-type', 'wheel'],
            obj=mock_context,
        )

    assert result.exit_code == 0

    run_mock = mock_context.build_libs.RunCommand.return_value
    _, kwargs = run_mock.run_command.call_args
    passed_env = kwargs.get('env', {})

    assert "_PYTHON_HOST_PLATFORM" not in passed_env
    assert "ARCHFLAGS" not in passed_env


@pytest.mark.parametrize(
    "machine,host_platform,archflags",
    [
        ("x86_64", "macosx-14.0-x86_64", "-arch x86_64"),
        ("arm64", "macosx-14.0-arm64", "-arch arm64"),
    ],
)
@patch("dev_python.platform.system", return_value="Darwin")
def test_build_python_wheel_macos_sets_arch_env(mock_system, machine, host_platform, archflags, mock_context):
    with patch("dev_python.platform.machine", return_value=machine):
        runner = CliRunner()
        result = runner.invoke(python_build, ["--output-type", "wheel"], obj=mock_context)
    assert result.exit_code == 0
    _, kwargs = mock_context.build_libs.RunCommand.return_value.run_command.call_args
    passed_env = kwargs.get("env", {})
    assert passed_env["_PYTHON_HOST_PLATFORM"] == host_platform
    assert passed_env["ARCHFLAGS"] == archflags


def test_build_python_system_libs(mock_context):
    """Test --use-system-libs flag prevents vcpkg env vars."""
    # Simulate the flag being processed by build_options
    mock_context.build_options.use_system_libs = True

    runner = CliRunner()
    result = runner.invoke(python_build, ['--use-system-libs'], obj=mock_context)

    assert result.exit_code == 0

    run_mock = mock_context.build_libs.RunCommand.return_value
    _, kwargs = run_mock.run_command.call_args
    passed_env = kwargs.get('env', {})

    # Ensure VCPKG keys are NOT present
    assert "VCPKG_ROOT" not in passed_env
    assert "VCPKG_MANIFEST_MODE" not in passed_env


@patch("dev_python.glob.glob")
def test_build_python_profiling(mock_glob, mock_context, clang_build_analyzer_checker):
    """Test profiling workflow: clang checks, analyzer start/stop, globbing."""
    # Setup glob to return dummy json files
    mock_glob.return_value = ["trace1.json", "trace2.json"]

    # Run the mock build with profiling
    runner = CliRunner()
    result = runner.invoke(python_build, ['--profile-build'], obj=mock_context)

    assert result.exit_code == 0

    # Check tool validation
    mock_context.build_libs.check_for_tool.assert_any_call("clang")
    mock_context.build_libs.check_for_tool.assert_any_call("ClangBuildAnalyzer")

    run_mock = mock_context.build_libs.RunCommand.return_value

    expected_cap_file = os.path.join(mock_context.python_build_dir, "capture")
    clang_build_analyzer_checker(run_mock.run_command.call_args_list,
                                 mock_context.python_build_dir,
                                 expected_cap_file)

    # Verify Env vars for clang
    _, kwargs = run_mock.run_command.call_args_list[1]
    passed_env = kwargs.get('env', {})
    assert passed_env["CMAKE_CXX_COMPILER"] == "clang++"
    assert passed_env["CMAKE_CXX_FLAGS"] == "-ftime-trace"

    # Verify JSON combining
    mock_context.build_libs.perf_json_combine.assert_called_once()


def test_build_python_failure(mock_context):
    """Test that exceptions during build are caught and raised as RuntimeError."""
    run_mock = mock_context.build_libs.RunCommand.return_value
    # Simulate a build failure
    run_mock.run_command.side_effect = Exception("Build failed")

    runner = CliRunner()
    result = runner.invoke(python_build, [], obj=mock_context)

    assert result.exit_code != 0
    assert isinstance(result.exception, RuntimeError)
    assert "Please check the output for details" in str(result.exception)


@pytest.fixture
def test_context(mock_context):
    """Extend context with test-specific fields."""
    mock_context.internal_test_data_dir = os.path.join(mock_context.sdk_dir, "data")
    mock_context.internal_test_data_tag = "v1"

    return mock_context


def test_python_test_basic(test_context):
    """Test running only unit tests (no test data provided/found)."""
    runner = CliRunner()
    result = runner.invoke(python_test, ['--skip-integration-tests',
                                         '--threads',
                                         str(test_context.build_options.threads)],
                           obj=test_context)

    assert result.exit_code == 0

    # Verify we checked for pytest-xdist
    test_context.build_libs.check_for_python_libs.assert_any_call(
        [("xdist", "pytest-xdist"), ("pytest_asyncio", "pytest-asyncio")]
    )

    run_mock = test_context.build_libs.RunCommand.return_value

    assert run_mock.run_command.call_count == 1

    expected_args = [
        sys.executable, "-m", "pytest", '-n', str(test_context.build_options.threads)
    ]
    args, kwargs = run_mock.run_command.call_args
    assert list(args) == expected_args
    assert kwargs['cwd'] == os.path.join(test_context.sdk_dir, "python", "tests")


def test_python_test_with_integration(test_context):
    """Test running unit tests AND integration tests when data dir exists."""
    runner = CliRunner()
    result = runner.invoke(python_test, [], obj=test_context)

    assert result.exit_code == 0

    run_mock = test_context.build_libs.RunCommand.return_value

    # Expect TWO run commands (Unit + Integration)
    assert run_mock.run_command.call_count == 2

    # Check 1st call (Unit)
    args, kwargs = run_mock.run_command.call_args_list[0]
    assert kwargs['cwd'] == os.path.join(test_context.sdk_dir, "python", "tests")

    # Check 2nd call (Integration)
    args, kwargs = run_mock.run_command.call_args_list[1]

    # Should use the constructed path from internal_test_data_dir + tag
    expected_data_dir = os.path.join(test_context.internal_test_data_dir,
                                     test_context.internal_test_data_tag)

    assert kwargs['env']['TEST_DATA_DIR'] == expected_data_dir
    assert kwargs['cwd'] == os.path.join(test_context.sdk_dir, "tests", "integration")


def test_python_test_explicit_data_dir(test_context):
    """Test passing --test-data-dir CLI argument."""
    custom_dir = "/tmp/custom_data"
    runner = CliRunner()
    result = runner.invoke(python_test, ['--test-data-dir', custom_dir], obj=test_context)

    assert result.exit_code == 0

    run_mock = test_context.build_libs.RunCommand.return_value
    assert run_mock.run_command.call_count == 2

    # Verify the env var uses the CLI arg, not the context internal dir
    _, kwargs = run_mock.run_command.call_args_list[1]
    assert kwargs['env']['TEST_DATA_DIR'] == custom_dir


def test_integration_failure(test_context):
    """Test failure during integration tests raises RuntimeError."""
    run_mock = test_context.build_libs.RunCommand.return_value

    # First call succeeds (unit), second fails (integration)
    run_mock.run_command.side_effect = [None, Exception("Integration Fail")]

    runner = CliRunner()
    result = runner.invoke(python_test, [], obj=test_context)

    assert result.exit_code != 0
    assert isinstance(result.exception, RuntimeError)
    assert "Please check the output for details" in str(result.exception)


def test_build_python_vcpkg_args(mock_context):
    """
    Test VCPKG-specific arguments:
    --vcpkg-toolchain, --vcpkg-triplet, --threads
    """
    runner = CliRunner()
    result = runner.invoke(python_build, [
        '--vcpkg-toolchain', '/tmp/custom_toolchain.cmake',
        '--vcpkg-triplet', 'arm64-osx',
        '--threads', '8'
    ], obj=mock_context)

    assert result.exit_code == 0

    # 1. Verify arguments were passed to the config processor
    mock_context.build_options.process_args.assert_called_once()
    call_kwargs = mock_context.build_options.process_args.call_args[1]
    assert call_kwargs['vcpkg_toolchain'] == '/tmp/custom_toolchain.cmake'
    assert call_kwargs['vcpkg_triplet'] == 'arm64-osx'
    assert call_kwargs['threads'] == '8'

    # 2. Verify Env vars were populated correctly in the run command
    run_mock = mock_context.build_libs.RunCommand.return_value
    _, kwargs = run_mock.run_command.call_args
    passed_env = kwargs.get('env', {})

    assert passed_env['VCPKG_TOOLCHAIN_FILE'] == '/tmp/custom_toolchain.cmake'
    assert passed_env['VCPKG_TARGET_TRIPLET'] == 'arm64-osx'
    assert passed_env['VCPKG_MAX_CONCURRENCY'] == '8'
    assert passed_env['OUSTER_SDK_BUILD_JOBS'] == '8'


def test_build_python_debug_and_no_manifest(mock_context):
    """
    Test build configuration flags:
    --build-type (Debug) and --no-manifest-mode
    """
    runner = CliRunner()
    result = runner.invoke(python_build, [
        '--build-type', 'Debug',
        '--no-manifest-mode'
    ], obj=mock_context)

    assert result.exit_code == 0

    run_mock = mock_context.build_libs.RunCommand.return_value
    _, kwargs = run_mock.run_command.call_args
    passed_env = kwargs.get('env', {})

    assert passed_env['BUILD_TYPE'] == 'Debug'

    call_kwargs = mock_context.build_options.process_args.call_args[1]
    assert call_kwargs['manifest_mode'] is False

    assert passed_env['VCPKG_MANIFEST_MODE'] == 'OFF'


def test_build_python_logging(mock_context):
    """
    Test logging and coverage flags:
    --cmake-log-file, --coverage-flags
    """
    runner = CliRunner()
    log_path = "/tmp/cmake_debug.log"

    result = runner.invoke(python_build, [
        '--cmake-log-file', log_path
    ], obj=mock_context)

    assert result.exit_code == 0

    run_mock = mock_context.build_libs.RunCommand.return_value
    _, kwargs = run_mock.run_command.call_args
    passed_env = kwargs.get('env', {})

    assert passed_env['OUSTER_SDK_CMAKE_LOG_FILE'] == log_path


@patch("dev_python.glob.glob")
def test_build_python_profiling_failure(mock_glob, mock_context):
    """
    Test that profiling handles build failures gracefully by
    stopping the analyzer before raising the error.
    """
    # Setup mocks
    mock_glob.return_value = []
    run_mock = mock_context.build_libs.RunCommand.return_value

    # Sequence of effects:
    # 1. ClangBuildAnalyzer --start (Succeeds)
    # 2. pip install ... (Fails)
    # 3. ClangBuildAnalyzer --stop (Succeeds, called in exception handler)
    run_mock.run_command.side_effect = [None, Exception("Compiler Error"), None]

    runner = CliRunner()
    # Run with --profile-build enabled
    result = runner.invoke(python_build, ['--profile-build'], obj=mock_context)

    # Assert the command failed as expected
    assert result.exit_code != 0

    # Verify logic flow:
    # We expect 'start' was called, then the 'build' (which failed),
    # and finally 'stop' should be called inside the exception handler.

    # Filter calls to ClangBuildAnalyzer
    analyzer_calls = [
        args[0] for args, _ in run_mock.run_command.call_args_list
        if args[0] == "ClangBuildAnalyzer"
    ]

    assert "ClangBuildAnalyzer" in analyzer_calls

    # Check that we specifically attempted to stop it
    # The arguments are usually ("ClangBuildAnalyzer", "--stop", ...)
    stop_called = any(
        call_args[0][1] == "--stop"
        for call_args in run_mock.run_command.call_args_list
        if len(call_args[0]) > 1
    )
    assert stop_called, "Analyzer stop command was not triggered on build failure"


def test_python_test_env_var_fallback(test_context):
    """
    Test that TEST_DATA_DIR is picked up from the OS environment
    if not provided via CLI or Context.
    """
    # 1. Clear context data to force fallback to Env Var
    test_context.internal_test_data_dir = None

    env_path = "/path/from/env/var"

    # 2. Inject environment variable
    with patch.dict(os.environ, {"TEST_DATA_DIR": env_path}):
        runner = CliRunner()
        result = runner.invoke(python_test, [], obj=test_context)

    assert result.exit_code == 0

    run_mock = test_context.build_libs.RunCommand.return_value

    # Should run Unit tests (1) + Integration tests (2)
    assert run_mock.run_command.call_count == 2

    # Inspect the Integration Test run (the 2nd call)
    _, kwargs = run_mock.run_command.call_args_list[1]

    # Verify the env var was passed into the subprocess
    assert kwargs['env']['TEST_DATA_DIR'] == env_path


@pytest.mark.parametrize("build_type", ["Debug", "RelWithDebInfo"])
def test_build_python_build_types(mock_context, build_type):
    """Test all non-default build types pass through to env correctly."""
    runner = CliRunner()
    result = runner.invoke(python_build, ['--build-type', build_type], obj=mock_context)

    assert result.exit_code == 0

    run_mock = mock_context.build_libs.RunCommand.return_value
    _, kwargs = run_mock.run_command.call_args
    passed_env = kwargs.get('env', {})

    assert passed_env['BUILD_TYPE'] == build_type
