import os
import sys
import pytest
import subprocess
import json
from unittest.mock import MagicMock, patch, mock_open, ANY

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import build_libs # noqa E402


@pytest.fixture
def mock_run_command():
    """Patches the generic run_command method to avoid actual subprocess calls."""
    with patch('build_libs.RunCommand.run_command') as mock:
        yield mock


@pytest.fixture
def mock_subprocess():
    """Patches subprocess.Popen for testing RunCommand directly."""
    with patch('subprocess.Popen') as mock:
        process_mock = MagicMock()
        process_mock.communicate.return_value = ("stdout output", "stderr output")
        process_mock.returncode = 0
        mock.return_value = process_mock
        yield mock


def test_os_independent_shlex_split_win():
    with patch('sys.platform', 'win32'):
        # On Windows, posix=0 keeps backslashes
        res = build_libs.os_independent_shlex_split("C:\\Path\\To\\File")
        assert res == ["C:\\Path\\To\\File"]


def test_os_independent_shlex_split_linux():
    with patch('sys.platform', 'linux'):
        res = build_libs.os_independent_shlex_split("/path/to/file")
        assert res == ["/path/to/file"]


def test_venv_folder_template():
    res = build_libs.venv_folder_template("/usr/bin/python 3.9", "/tmp")
    # Spaces replaced by underscores, basename used
    assert res == os.path.join("/tmp", "venv-python_3.9")


def test_individual_timer():
    with patch('time.time') as mock_time:
        mock_time.side_effect = [90.0, 100.0, 165.0]
        timer = build_libs.IndividualTimer("test")
        timer.start()
        timer.stop()

        assert timer.get_duration() == 65.0
        assert "1 minutes 5 seconds" in str(timer)


def test_timer_error_if_not_stopped():
    timer = build_libs.IndividualTimer("test")
    with pytest.raises(Exception, match="Need call stop"):
        timer.get_duration()


def test_build_timer_collision_avoidance():
    """Duplicate labels must produce distinct timers with counter suffixes."""
    bt = build_libs.BuildTimer()
    t1 = bt.make_timer("build")
    t2 = bt.make_timer("build")
    t3 = bt.make_timer("build")
    assert t1 is not t2
    assert t2 is not t3
    keys = list(bt._timers.keys())
    assert "build" in keys
    assert any("#2" in k for k in keys)
    assert any("#3" in k for k in keys)


def test_run_command_stderr_always_printed(mock_subprocess, capsys):
    """Stderr should be printed even when the process exits successfully."""
    proc = mock_subprocess.return_value
    proc.communicate.return_value = ("", "warning: something")
    proc.returncode = 0

    runner = build_libs.RunCommand()
    runner.run_command("mybin", cwd="/tmp")

    captured = capsys.readouterr()
    assert "warning: something" in captured.out


def test_check_for_python_lib_error_message(capsys):
    """Error message must include 'needs to be installed'."""
    result = build_libs.check_for_python_lib("nonexistent_pkg_xyz", fail_on_missing=False)
    assert result is False
    captured = capsys.readouterr()
    assert "needs to be installed" in captured.out


def test_defaults_lazy_evaluation():
    """Defaults.default_artifact_dir/build_dir must reflect cwd at call time."""
    with patch("os.getcwd", return_value="/fake/cwd"):
        assert build_libs.Defaults.default_artifact_dir() == os.path.join("/fake/cwd", "artifacts")
        assert build_libs.Defaults.default_build_dir() == os.path.join("/fake/cwd", "build")


def test_confirm_auto_fix_yes(monkeypatch):
    monkeypatch.setattr("builtins.input", lambda _: "y")
    assert build_libs.confirm_auto_fix() is True


def test_confirm_auto_fix_no(monkeypatch):
    monkeypatch.setattr("builtins.input", lambda _: "n")
    assert build_libs.confirm_auto_fix() is False


def test_run_command_success(mock_subprocess):
    runner = build_libs.RunCommand()
    output = runner.run_command("echo", "hello", cwd="/tmp")

    assert output == "stdout output"
    mock_subprocess.assert_called_with(
        ("echo", "hello"),
        env=ANY,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=os.path.normpath("/tmp"),
        text=True,
        bufsize=1
    )


def test_run_command_failure(mock_subprocess):
    # Simulate failure
    mock_subprocess.return_value.returncode = 1

    runner = build_libs.RunCommand()
    with pytest.raises(Exception, match="Error Running Process"):
        runner.run_command("bad_command")


def test_run_command_tty(mock_subprocess):
    runner = build_libs.RunCommand(tty=True)
    runner.run_command("ls")

    # When tty is True, stdout/stderr map to sys.stdout
    mock_subprocess.assert_called_with(
        ("ls",),
        env=ANY,
        stdout=sys.stdout,
        stderr=subprocess.STDOUT,
        cwd=ANY,
        text=True,
        bufsize=1
    )


@patch('build_libs.os_independent_shlex_split', side_effect=lambda x: [x])
def test_python_init_and_venv(mock_split, mock_run_command):
    # Mock os.path.exists to return False so it tries to make venv
    with patch('os.path.exists', return_value=False):
        # We also need to mock ensure_uv_installed inside init
        with patch.object(build_libs.Python, 'ensure_uv_installed'):
            _ = build_libs.Python("python3", "/tmp/venv")

    # Check if venv creation was called
    # run_command called with args list
    # The first call should be creation of venv
    args, _ = mock_run_command.call_args_list[0]
    assert args[0] == "python3"
    assert args[1] == "-m"
    assert args[2] == "venv"


def test_python_install_deps(mock_run_command):
    # Mocking init to skip venv creation/checks
    with patch.object(build_libs.Python, 'make_venv'), \
         patch.object(build_libs.Python, 'ensure_uv_installed'):
        py = build_libs.Python("python3", "/tmp/venv")

        py.install_pip_deps("numpy", "pandas")

        # Verify call to run_command (which mocks running uv pip install)
        # Note: Python.run_python_command prepends the python bin
        call_args = mock_run_command.call_args[0]
        # Flatten the args tuple
        cmd = list(call_args)
        assert "uv" in cmd
        assert "pip" in cmd
        assert "install" in cmd
        assert "numpy" in cmd


def test_cmake_ccache_detection(mock_run_command):
    with patch.dict(os.environ, {"CCACHE_DIR": "/tmp/ccache"}), \
         patch('shutil.which', return_value="/usr/bin/ccache"):

        cmake = build_libs.CMake("/src", "/build", "/artifacts")
        # Access private member to check if arg was added
        assert "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache" in cmake._cmake_args


def test_cmake_generate_and_build(mock_run_command):
    cmake = build_libs.CMake("/src", "/build", "/artifacts", cmake_path="cmake")

    # Test Generate
    cmake.generate()
    gen_args = mock_run_command.call_args[0]
    assert "-B" in gen_args
    assert "/build" in gen_args
    assert "-S" in gen_args

    # Test Build
    cmake.build(targets=["install"])
    build_args = mock_run_command.call_args[0]
    assert "--build" in build_args
    assert "--target" in build_args
    assert "install" in build_args


@patch('glob.glob')
@patch('shutil.copy')
def test_cmake_package(mock_copy, mock_glob, mock_run_command):
    cmake = build_libs.CMake("/src", "/build", "/artifacts")

    # Mock finding a tarball
    mock_glob.return_value = ["/build/ouster-sdk.tgz"]

    cmake.make_package(prefix="/tmp/stage")

    # Verify build command for package target
    build_args = mock_run_command.call_args[0]
    assert "--target" in build_args
    assert "package" in build_args

    # 3 glob patterns × 1 match each × 2 destinations (artifact_dir + prefix) = 6 copies
    assert mock_copy.call_count == 6


def test_doxygen_generate(mock_run_command):
    with patch('builtins.open', mock_open(read_data="PROJECT = $project")) as m_open:
        doxy = build_libs.Doxygen("MyProject", "1.0", "/out", "/src")
        doxy.generate_doxygen()

        # Check if Doxygen binary was called
        args = mock_run_command.call_args[0]
        assert args[0] == "doxygen"

        # Verify config file generation (write)
        handle = m_open()
        handle.write.assert_called()
        written_content = handle.write.call_args[0][0]
        assert "PROJECT = MyProject" in written_content


def test_doxygen_get_warnings():
    """operator= lines must be filtered; real warnings must be kept."""
    log_content = "file.cpp:10: warning: something bad happened\nfile.cpp:12: warning: operator= ignored\n"
    with patch('builtins.open', mock_open(read_data=log_content)), \
         patch('os.path.exists', return_value=True):

        doxy = build_libs.Doxygen("P", "1", "/out", "/src", warning_log="/log.txt")
        warnings = doxy.get_warnings()

        assert len(warnings) == 1
        assert "something bad happened" in warnings[0]
        assert not any("operator=" in w for w in warnings)


def test_doxygen_get_warnings_missing_log(capsys):
    """A missing log file must print an error and return None."""
    with patch('os.path.exists', return_value=False):
        doxy = build_libs.Doxygen("P", "1", "/out", "/src", warning_log="/missing.txt")
        result = doxy.get_warnings()

    assert result is None
    assert "not found" in capsys.readouterr().out


def test_clang_init_missing_lib():
    """Verify behavior if clang can't be imported (should ideally raise or fail gracefully depending on usage)."""
    with patch.dict(sys.modules):
        if 'clang.cindex' in sys.modules:
            del sys.modules['clang.cindex']
        with patch('builtins.__import__', side_effect=ImportError):
            with pytest.raises(ImportError):
                build_libs.Clang()


def test_check_for_tool_found():
    with patch('shutil.which', return_value="/usr/bin/tool"):
        assert build_libs.check_for_tool("tool") == "/usr/bin/tool"


def test_check_for_tool_missing():
    with patch('shutil.which', return_value=None):
        with pytest.raises(RuntimeError):
            build_libs.check_for_tool("tool")


def test_parse_version():
    cmake_content = 'set(OusterSDK_VERSION_STRING 2.1.0)'
    with patch('builtins.open', mock_open(read_data=cmake_content)):
        version = build_libs.parse_version("/path/to/sdk")
        assert version == "2.1.0"


def test_perf_json_combine(tmp_path):
    f1 = tmp_path / "1.json"
    f2 = tmp_path / "2.json"
    out = tmp_path / "out.json"

    with open(f1, 'w') as f:
        json.dump({"traceEvents": [{"a": 1}]}, f)
    with open(f2, 'w') as f:
        json.dump({"traceEvents": [{"b": 2}]}, f)

    build_libs.perf_json_combine([str(f1), str(f2)], str(out))

    with open(out, 'r') as f:
        data = json.load(f)
        assert len(data['traceEvents']) == 2


@patch('subprocess.check_output')
def test_get_env_from_sourced_shell(mock_sub, tmp_path):
    script = tmp_path / "env.sh"
    script.touch()

    # subprocess output needs to be a valid JSON string of an env dict
    mock_env = {"MY_VAR": "VALUE"}
    mock_sub.return_value = json.dumps(mock_env)

    res = build_libs.get_env_from_sourced_shell(str(script))
    assert res["MY_VAR"] == "VALUE"


@patch('build_libs.CMake')
@patch('glob.glob')
@patch('shutil.copy')
@patch('os.makedirs')
def test_generate_compile_commands(mock_mkdirs, mock_copy, mock_glob, mock_cmake_cls):
    # Setup mocks
    mock_cmake = mock_cmake_cls.return_value
    mock_glob.return_value = ["/build/compile_commands.json"]
    mock_mkdirs.return_value = True
    build_libs.generate_compile_commands(
        output="/tmp/out.json",
        sdk_dir="/src",
        artifact_dir="/art",
        build_dir="/build"
    )

    # Check CMake Config
    mock_cmake_cls.assert_called()
    call_kwargs = mock_cmake_cls.call_args[1]
    assert "-DBUILD_EXAMPLES=ON" in call_kwargs['cmake_args']

    # Check execution
    mock_cmake.generate.assert_called()
    mock_cmake.build.assert_called_with(targets=["cpp_gen"])

    # Check copy
    mock_copy.assert_called()


def test_check_for_tool_custom_path_takes_priority():
    """An explicit tool_path that exists must be returned without consulting PATH."""
    with patch("os.path.exists", return_value=True), \
         patch("shutil.which") as mock_which:
        result = build_libs.check_for_tool("cmake", tool_path="/custom/bin/cmake")
    assert result == "/custom/bin/cmake"
    mock_which.assert_not_called()


def test_check_for_tool_falls_back_to_which():
    """When tool_path is not given, shutil.which must be consulted."""
    with patch("os.path.exists", return_value=False), \
         patch("shutil.which", return_value="/usr/bin/cmake") as mock_which:
        result = build_libs.check_for_tool("cmake")
    assert result == "/usr/bin/cmake"
    mock_which.assert_called_with("cmake")


def test_check_for_tool_missing_returns_none_when_not_required():
    """fail_on_missing=False must return None instead of raising."""
    with patch("os.path.exists", return_value=False), \
         patch("shutil.which", return_value=None):
        result = build_libs.check_for_tool("nonexistent", fail_on_missing=False)
    assert result is None


def test_check_for_tool_missing_raises_when_required():
    """fail_on_missing=True (default) must raise RuntimeError with install hint."""
    with patch("os.path.exists", return_value=False), \
         patch("shutil.which", return_value=None):
        with pytest.raises(RuntimeError, match="needs to be installed"):
            build_libs.check_for_tool("nonexistent")


# ---------------------------------------------------------------------------
# initialize_vcpkg — first-run clone vs subsequent pull
# ---------------------------------------------------------------------------

def test_initialize_vcpkg_clones_on_first_run(tmp_path):
    """When vcpkg_dir does not exist, the repo must be cloned and bootstrapped."""
    vcpkg_dir = tmp_path / "vcpkg"

    mock_repo = MagicMock()
    with patch("os.path.exists", return_value=False), \
         patch("git.Repo.clone_from", return_value=mock_repo) as mock_clone, \
         patch("subprocess.check_output") as mock_sub, \
         patch("os.name", "posix"):
        build_libs.initialize_vcpkg(str(vcpkg_dir))

    mock_clone.assert_called_once_with(
        "https://github.com/microsoft/vcpkg.git", str(vcpkg_dir))
    mock_sub.assert_called_once()
    assert "bootstrap-vcpkg.sh" in mock_sub.call_args[0][0][0]


def test_initialize_vcpkg_pulls_on_subsequent_run(tmp_path):
    """When vcpkg_dir already exists, origin must be fetched and reset --hard."""
    vcpkg_dir = tmp_path / "vcpkg"
    vcpkg_dir.mkdir()

    mock_repo = MagicMock()
    mock_repo.active_branch.name = "master"
    with patch("os.path.exists", return_value=True), \
         patch("git.Repo", return_value=mock_repo) as mock_repo_cls:
        build_libs.initialize_vcpkg(str(vcpkg_dir))

    mock_repo_cls.assert_called_once_with(str(vcpkg_dir))
    mock_repo.remotes.origin.fetch.assert_called_once()
    mock_repo.git.reset.assert_called_once_with("--hard", "origin/master")


def test_initialize_vcpkg_reinit_deletes_and_reclones(tmp_path):
    """reinit=True must wipe the existing directory and clone fresh."""
    vcpkg_dir = tmp_path / "vcpkg"
    vcpkg_dir.mkdir()

    mock_repo = MagicMock()
    # os.path.exists call sequence:
    #   1. rmtree_readonly existence guard  → True  (dir present, proceed with rmtree)
    #   2. initialize_vcpkg reinit guard    → True  (triggers _force_rmtree path)
    #   3. initialize_vcpkg clone guard     → False (dir gone after wipe, so clone runs)
    with patch("shutil.rmtree") as mock_rmtree, \
         patch("os.path.exists", side_effect=[True, True, False]), \
         patch("git.Repo.clone_from", return_value=mock_repo), \
         patch("subprocess.check_output"), \
         patch("os.name", "posix"):
        build_libs.initialize_vcpkg(str(vcpkg_dir), reinit=True)

    assert mock_rmtree.call_count == 1
    assert mock_rmtree.call_args[0][0] == str(vcpkg_dir)
