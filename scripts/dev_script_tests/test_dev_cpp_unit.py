import os
import pytest
import sys

from unittest.mock import patch
from click.testing import CliRunner

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "dev_script_library")))

import dev_cpp  # noqa E420


def test_build_cpp_basic(mock_context):
    """Test standard Release build command invocation."""
    runner = CliRunner()

    result = runner.invoke(dev_cpp.cpp_build,
                           ['--build-type', 'Release'],
                           obj=mock_context)

    assert result.exit_code == 0

    mock_context.build_options.run_vcpkg_initialized_check.assert_called_once()

    cmake_cls = mock_context.build_libs.CMake
    _, kwargs = cmake_cls.call_args
    kwargs = kwargs['cmake_args']

    assert "-DBUILD_TESTING=ON" in kwargs
    assert "-DVCPKG_MANIFEST_MODE=ON" in kwargs
    assert "-DOUSTER_INTERNAL_TESTS=ON" in kwargs
    assert "-DOUSTER_EXTERNAL_TESTS=ON" in kwargs

    # Check path-dependent args
    expected_toolchain = f"-DCMAKE_TOOLCHAIN_FILE={mock_context.build_options.vcpkg_toolchain}"
    assert expected_toolchain in kwargs

    cmake_instance = mock_context.build_libs.CMake.return_value
    cmake_instance.build.assert_called_with(threads=mock_context.build_options.threads)
    assert cmake_instance.generate.call_count == 1


def test_build_cpp_package(mock_context):
    """Test packaging mode."""
    runner = CliRunner()
    install_dir = os.path.join("tmp", "install")

    result = runner.invoke(dev_cpp.cpp_build,
                           ['--package', '--install-dir', install_dir],
                           obj=mock_context)
    assert result.exit_code == 0

    cmake_instance = mock_context.build_libs.CMake.return_value
    kwargs = mock_context.build_libs.CMake.call_args_list[0].kwargs['cmake_args']
    assert "-DBUILD_SHARED_LIBRARY=ON" in kwargs
    assert "-DBUILD_TESTING=OFF" in kwargs

    kwargs = mock_context.build_libs.CMake.call_args_list[1].kwargs['cmake_args']
    assert "-DOUSTER_SDK_TEST_SOURCE=DYNAMIC" in kwargs
    assert "-DOUSTER_INTERNAL_TESTS=OFF" in kwargs
    assert "-DOUSTER_EXTERNAL_TESTS=ON" in kwargs

    cmake_instance.make_package.assert_called_with(prefix=install_dir)
    cmake_instance.build.assert_called_with(threads=mock_context.build_options.threads)
    # Two CMake configure+build cycles: one for the package, one for the shared-library test build.
    assert cmake_instance.generate.call_count == 2
    assert cmake_instance.build.call_count == 2
    assert cmake_instance.install.call_count == 2


def test_build_cpp_profile(mock_context, clang_build_analyzer_checker):
    """Test profiling mode flags."""
    runner = CliRunner()
    result = runner.invoke(dev_cpp.cpp_build,
                           ['--profile-build'],
                           obj=mock_context)

    assert result.exit_code == 0

    mock_context.build_libs.check_for_tool.assert_any_call("clang")

    expected_cap_file = os.path.join(mock_context.cmake_build_dir, "capture")
    run_cmd = mock_context.build_libs.RunCommand.return_value

    clang_build_analyzer_checker(run_cmd.run_command.call_args_list,
                                 mock_context.cmake_build_dir,
                                 expected_cap_file)


def test_cpp_test_command(mock_context):
    """Test the test runner command."""
    runner = CliRunner()

    mock_context.build_libs.check_for_tool.return_value = os.path.abspath("ctest")

    result = runner.invoke(dev_cpp.cpp_test, ['--build-type', 'Debug'], obj=mock_context)

    assert result.exit_code == 0

    run_cmd_instance = mock_context.build_libs.RunCommand.return_value
    args, kwargs = run_cmd_instance.run_command.call_args

    assert args[0] == os.path.abspath("ctest")
    assert "--output-on-failure" in args
    assert kwargs['cwd'] == mock_context.cmake_build_dir


def test_compile_commands(mock_context):
    """Test compile_commands generation."""
    runner = CliRunner()

    result = runner.invoke(dev_cpp.compile_commands, [], obj=mock_context)

    assert result.exit_code == 0

    expected_output = os.path.join(mock_context.sdk_dir, "compile_commands.json")

    mock_context.build_libs.generate_compile_commands.assert_called_once()
    args, kwargs = mock_context.build_libs.generate_compile_commands.call_args

    assert args[0] == expected_output
    assert kwargs['toolchain'] == mock_context.build_options.vcpkg_toolchain


def test_find_latest_package():
    """Test the helper function to find latest zip (independent of context)."""
    with patch("glob.glob") as mock_glob, \
         patch("os.path.getctime") as mock_time:

        f1 = os.path.join("tmp", "ouster-sdk-1.zip")
        f2 = os.path.join("tmp", "ouster-sdk-2.zip")

        mock_glob.return_value = [f1, f2]

        mock_time.side_effect = lambda x: 100 if "sdk-1" in x else 200

        latest = dev_cpp.find_latest_package("tmp")
        assert latest == f2

    with patch("glob.glob", return_value=[]):
        with pytest.raises(FileNotFoundError):
            dev_cpp.find_latest_package("tmp")


@pytest.mark.parametrize("cli_args, expected_cmake_flags", [
    (['--no-examples'],
     ["-DBUILD_EXAMPLES=OFF"]),
    (['--hil-examples'],
     ["-DRUN_HIL_EXAMPLES=ON"]),
    (['--no-manifest-mode'],
     ["-DVCPKG_MANIFEST_MODE=OFF"]),
    (['--no-tests'],
     ["-DBUILD_TESTING=OFF"]),
])
def test_build_cpp_toggles(mock_context, cli_args, expected_cmake_flags):
    """Test boolean toggle flags and their impact on CMake arguments."""
    runner = CliRunner()
    result = runner.invoke(dev_cpp.cpp_build, cli_args, obj=mock_context)

    assert result.exit_code == 0

    cmake_cls = mock_context.build_libs.CMake
    _, kwargs = cmake_cls.call_args
    actual_cmake_args = kwargs['cmake_args']

    for flag in expected_cmake_flags:
        assert flag in actual_cmake_args

    if '--no-tests' in cli_args:
        assert not any("-DOUSTER_EXTERNAL_TESTS=ON" in arg for arg in actual_cmake_args)
        assert not any("-DOUSTER_INTERNAL_TESTS=ON" in arg for arg in actual_cmake_args)


def test_build_cpp_system_libs(mock_context):
    """Test that --use-system-libs suppresses vcpkg toolchain arguments."""
    runner = CliRunner()
    result = runner.invoke(dev_cpp.cpp_build, ['--use-system-libs'], obj=mock_context)

    assert result.exit_code == 0

    cmake_cls = mock_context.build_libs.CMake
    _, kwargs = cmake_cls.call_args
    actual_cmake_args = kwargs['cmake_args']

    # Ensure toolchain file is NOT passed
    assert not any("-DCMAKE_TOOLCHAIN_FILE" in arg for arg in actual_cmake_args)
    assert not any("-DVCPKG_TARGET_TRIPLET" in arg for arg in actual_cmake_args)


def test_build_cpp_extra_args_and_env(mock_context):
    """Test passing extra cmake args and VCPKG_BINARY_SOURCES env injection."""
    runner = CliRunner()

    # Mock the environment variable inside the build options context
    mock_context.build_options.build_env["VCPKG_BINARY_SOURCES"] = "clear;files,/tmp/vcpkg"

    result = runner.invoke(dev_cpp.cpp_build,
                           ['--cmake-arg', '-DUSER_CUSTOM_FLAG=123',
                            '--cmake-arg', '-DANOTHER_FLAG=ON'],
                           obj=mock_context)

    assert result.exit_code == 0

    cmake_cls = mock_context.build_libs.CMake
    _, kwargs = cmake_cls.call_args
    actual_cmake_args = kwargs['cmake_args']

    # Check custom args
    assert "-DUSER_CUSTOM_FLAG=123" in actual_cmake_args
    assert "-DANOTHER_FLAG=ON" in actual_cmake_args

    # Check Environment injection
    assert "-DVCPKG_BINARY_SOURCES=clear;files,/tmp/vcpkg" in actual_cmake_args


def test_cpp_test_modes(mock_context, tmp_path):
    """Test variants of the test runner command."""
    runner = CliRunner()
    mock_context.build_libs.check_for_tool.return_value = "ctest"

    result = runner.invoke(dev_cpp.cpp_test, ['--use-shared-libs'], obj=mock_context)
    assert result.exit_code == 0

    run_cmd = mock_context.build_libs.RunCommand.return_value
    _, kwargs = run_cmd.run_command.call_args

    assert "shared_tests" in kwargs['cwd']

    custom_dir = tmp_path / "custom_test_dir"
    custom_dir.mkdir()
    custom_dir_str = str(custom_dir)

    result = runner.invoke(dev_cpp.cpp_test, ['--test-dir-override', custom_dir_str], obj=mock_context)

    # Debugging tip: if this fails, print result.output to see the click error
    assert result.exit_code == 0

    _, kwargs = run_cmd.run_command.call_args
    assert kwargs['cwd'] == custom_dir_str


def test_build_cpp_config_overrides(mock_context):
    """Test manual overrides for binaries, triplets, and threads."""
    runner = CliRunner()

    custom_cmake = "/opt/bin/cmake"
    custom_triplet = "x64-linux-custom"
    custom_threads = "42"

    result = runner.invoke(dev_cpp.cpp_build,
                           ['--cmake-bin', custom_cmake,
                            '--vcpkg-triplet', custom_triplet,
                            '--threads', custom_threads],
                           obj=mock_context)

    assert result.exit_code == 0

    # 1. Verify CMake Binary Override
    # It should have checked for the custom tool path
    mock_context.build_libs.check_for_tool.assert_any_call(custom_cmake)
    # The CMake class should be initialized with the custom binary
    args, kwargs = mock_context.build_libs.CMake.call_args
    assert kwargs['cmake_path'] == custom_cmake

    # 2. Verify Triplet Override
    cmake_args = kwargs['cmake_args']
    assert f"-DVCPKG_TARGET_TRIPLET={custom_triplet}" in cmake_args

    # 3. Verify Thread Override
    # Should be in cmake args (for vcpkg concurrency)
    assert f"-DVCPKG_MAX_CONCURRENCY={custom_threads}" in cmake_args
    # Should be passed to the build method
    cmake_instance = mock_context.build_libs.CMake.return_value
    cmake_instance.build.assert_called_with(threads=custom_threads)


def test_build_cpp_coverage_flags(mock_context):
    """Test that coverage flags inject the correct environment variable."""
    runner = CliRunner()

    # We must mock platform.system to ensure we are on Linux
    # because the script raises a RuntimeError if coverage is used on non-Linux
    with patch("platform.system", return_value="Linux"):
        result = runner.invoke(dev_cpp.cpp_build,
                               ['--coverage-flags'],
                               obj=mock_context)

    assert result.exit_code == 0

    # Verify the environment variable was set in the build context
    assert mock_context.build_options.build_env.get("CMAKE_COVERAGE_TESTS") == "true"


def test_cpp_test_custom_binary(mock_context):
    """Test overriding the ctest binary."""
    runner = CliRunner()
    custom_ctest = "/usr/local/bin/ctest"

    # Mock check_for_tool to return the custom path when asked
    mock_context.build_libs.check_for_tool.return_value = custom_ctest

    result = runner.invoke(dev_cpp.cpp_test, ['--ctest-bin', custom_ctest], obj=mock_context)

    assert result.exit_code == 0

    # Verify we asked to check the specific tool path
    mock_context.build_libs.check_for_tool.assert_called_with("ctest", tool_path=custom_ctest)

    # Verify the run command actually used it
    run_cmd = mock_context.build_libs.RunCommand.return_value
    args, _ = run_cmd.run_command.call_args
    assert args[0] == custom_ctest


def test_compile_commands_custom_output(mock_context):
    """Test overrides for compile_commands generation."""
    runner = CliRunner()
    custom_out = os.path.join("tmp", "custom_compile_commands.json")

    result = runner.invoke(dev_cpp.compile_commands, ['--output', custom_out], obj=mock_context)

    assert result.exit_code == 0

    mock_context.build_libs.generate_compile_commands.assert_called_once()
    args, _ = mock_context.build_libs.generate_compile_commands.call_args

    # args[0] is the output path
    assert args[0] == custom_out
