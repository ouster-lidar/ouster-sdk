import os
import sys
import pytest
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "dev_script_library")))

from context import ClickContext  # noqa E402


class TestClickContext:
    @pytest.fixture
    def mock_build_libs(self):
        return MagicMock()

    @pytest.fixture
    def ctx(self, mock_build_libs):
        """Creates a ClickContext instance with mocked paths."""
        # We patch os.path.abspath to return predictable strings
        with patch("os.path.abspath") as mock_abspath:
            mock_abspath.side_effect = lambda x: x if str(x).startswith("/") else f"/mock/abs/{x}"

            # We mock dirname specifically during init to set the base dev_dir
            with patch("context.os.path.dirname") as mock_dirname:
                mock_dirname.return_value = "/mock/dev/dir"
                ctx = ClickContext(build_libs=mock_build_libs)

            return ctx

    def test_init_directory_structures(self, ctx):
        assert ctx.dev_dir == "/mock/dev/dir"
        assert "sdk-extensions" in ctx.sdkx_dir
        assert ctx.submodules == {}

    def test_lazy_directory_creation(self, ctx):
        with patch("os.path.isdir", return_value=False), \
             patch("os.makedirs") as mock_makedirs:

            _ = ctx.cmake_build_dir
            mock_makedirs.assert_called_with(ctx._cmake_build_dir)

    def test_dev_persistent_dir_env_var(self):
        with patch.dict(os.environ, {"DEV_PERSISTENT_DIR": "/custom/path"}):
            ctx = ClickContext(MagicMock())
            assert ctx._dev_persistent_dir == "/custom/path"

    def test_add_module(self, ctx):
        """Test dynamic module importing."""
        # We use patch.object on sys to ensure we modify the exact module reference
        # We also patch dirname globally so the method call inside works correctly
        with patch("importlib.import_module") as mock_import, \
             patch.object(sys, "path", []) as mock_sys_path, \
             patch("os.path.dirname", side_effect=os.path.dirname):

            mock_module = MagicMock()
            mock_import.return_value = mock_module

            # Use a dummy path that looks like a file
            ctx.add_module("/path/to/plugin.py")

            # Verify the directory of the file was added to path
            assert "/path/to" in mock_sys_path
            assert "plugin" in ctx.submodules
            mock_module.import_module.assert_called_once_with(ctx)


class TestBuildOptions:
    @pytest.fixture
    def ctx_mock(self):
        ctx = MagicMock()
        ctx.vcpkg_dir = "/mock/vcpkg"
        return ctx

    def test_triplet_detection_linux(self, ctx_mock):
        with patch("platform.machine", return_value="x86_64"), \
             patch("platform.system", return_value="Linux"):

            opts = ClickContext.BuildOptions(ctx_mock.vcpkg_dir)
            assert opts.vcpkg_triplet == "x64-linux"

    def test_coverage_flags_linux_only(self, ctx_mock):
        """Ensure coverage flags raise error on non-Linux systems."""
        with patch("platform.system", return_value="Windows"):
            opts = ClickContext.BuildOptions(ctx_mock.vcpkg_dir)
            # Must call process_args manually as __init__ doesn't accept coverage_flags
            with pytest.raises(RuntimeError, match="Coverage flags are not supported"):
                opts.process_args(coverage_flags=True)

    def test_coverage_flags_success(self, ctx_mock):
        """Ensure coverage flags set env var on Linux."""
        with patch("platform.system", return_value="linux"):
            opts = ClickContext.BuildOptions(ctx_mock.vcpkg_dir)
            opts.process_args(coverage_flags=True)
            assert opts.build_env["CMAKE_COVERAGE_TESTS"] == "true"

    def test_threads_calculation(self, ctx_mock):
        with patch("os.cpu_count", return_value=8):
            opts = ClickContext.BuildOptions(ctx_mock.vcpkg_dir, threads=None)
            assert opts.threads == 4

        opts = ClickContext.BuildOptions(ctx_mock.vcpkg_dir, threads=12)
        assert opts.threads == 12

    def test_vcpkg_toolchain_autodiscovery_local(self, ctx_mock):
        """Test finding vcpkg toolchain inside the vcpkg_dir."""
        def side_effect(path):
            path = str(path).replace("\\", "/")
            return "/mock/vcpkg/scripts/buildsystems/vcpkg.cmake" in path or path == "/mock/vcpkg"

        with patch("os.path.exists", side_effect=side_effect), \
             patch("os.path.isdir", return_value=True):

            opts = ClickContext.BuildOptions(ctx_mock.vcpkg_dir)
            assert opts.vcpkg_toolchain is not None

    def test_vcpkg_toolchain_from_env(self, ctx_mock):
        """Test finding vcpkg toolchain from VCPKG_ROOT env var."""

        def exists_side_effect(path):
            if path == ctx_mock.vcpkg_dir:
                return True
            return False

        with patch.dict(os.environ, {"VCPKG_ROOT": "/env/vcpkg"}), \
             patch("os.path.exists", side_effect=exists_side_effect), \
             patch("os.path.isdir", return_value=True):

            opts = ClickContext.BuildOptions(ctx_mock.vcpkg_dir)

            expected = os.path.join("/env/vcpkg", "scripts", "buildsystems", "vcpkg.cmake")
            assert opts.vcpkg_toolchain == expected


class TestProcessArgsSentinel:
    """Verify that subsequent process_args calls do not clobber explicitly set values."""

    @pytest.fixture
    def opts(self):
        ctx = MagicMock()
        ctx.vcpkg_dir = "/mock/vcpkg"
        with patch("os.path.exists", return_value=False), \
             patch("os.path.isdir", return_value=False):
            return ClickContext.BuildOptions(ctx.vcpkg_dir)

    def test_explicit_toolchain_survives_threads_only_call(self, opts):
        """Setting toolchain then calling process_args(threads=N) must preserve it."""
        opts.vcpkg_toolchain = "/explicit/vcpkg.cmake"
        opts.process_args(threads=4)
        assert opts.vcpkg_toolchain == "/explicit/vcpkg.cmake"

    def test_explicit_triplet_survives_threads_only_call(self, opts):
        """Setting triplet then calling process_args(threads=N) must preserve it."""
        opts.vcpkg_triplet = "arm64-osx"
        opts.process_args(threads=4)
        assert opts.vcpkg_triplet == "arm64-osx"

    def test_explicit_toolchain_can_be_overridden(self, opts):
        """Passing a new toolchain explicitly must replace the old one."""
        opts.vcpkg_toolchain = "/old/vcpkg.cmake"
        with patch("os.path.exists", return_value=False):
            opts.process_args(vcpkg_toolchain="/new/vcpkg.cmake")
        assert opts.vcpkg_toolchain == "/new/vcpkg.cmake"

    def test_threads_only_call_does_not_trigger_autodiscovery(self, opts):
        """process_args(threads=N) must not overwrite a None toolchain via autodiscovery
        when no vcpkg dir or env var is present."""
        opts.vcpkg_toolchain = None
        with patch("os.path.exists", return_value=False), \
             patch("os.path.isdir", return_value=False), \
             patch.dict(os.environ, {}, clear=True):
            opts.process_args(threads=2)
        assert opts.vcpkg_toolchain is None


class TestVcpkgChecks:
    @pytest.fixture
    def ctx_mock(self):
        ctx = MagicMock()
        ctx.vcpkg_dir = "/mock/vcpkg"
        return ctx

    def test_run_vcpkg_check_success(self, ctx_mock):
        opts = ClickContext.BuildOptions(ctx_mock.vcpkg_dir)
        opts.vcpkg_toolchain = "/some/path"
        opts.run_vcpkg_initialized_check()

    def test_run_vcpkg_check_failure(self, ctx_mock, capsys):
        opts = ClickContext.BuildOptions(ctx_mock.vcpkg_dir)
        opts.vcpkg_toolchain = None
        opts.use_system_libs = False

        with patch("sys.argv", ["script.py"]):
            with pytest.raises(RuntimeError, match="No vcpkg toolchain"):
                opts.run_vcpkg_initialized_check()

        captured = capsys.readouterr()
        assert "ERROR: No source of dependencies set" in captured.out

    def test_run_vcpkg_check_shows_all_guidance_for_new_user(self, ctx_mock, capsys):
        """A brand-new user with no toolchain set must see all four remediation options."""
        opts = ClickContext.BuildOptions(ctx_mock.vcpkg_dir)
        opts.vcpkg_toolchain = None
        opts.use_system_libs = False

        with patch("sys.argv", ["dev.py"]):
            with pytest.raises(RuntimeError):
                opts.run_vcpkg_initialized_check()

        out = capsys.readouterr().out
        assert "enable-local-vcpkg" in out
        assert "VCPKG_ROOT" in out
        assert "--vcpkg-toolchain" in out
        assert "--use-system-libs" in out

    def test_run_vcpkg_check_local_only_hides_alternatives(self, ctx_mock, capsys):
        """local_vcpkg_only=True must suppress the VCPKG_ROOT / --vcpkg-toolchain / --use-system-libs lines."""
        opts = ClickContext.BuildOptions(ctx_mock.vcpkg_dir)
        opts.vcpkg_toolchain = None
        opts.use_system_libs = False

        with patch("sys.argv", ["dev.py"]):
            with pytest.raises(RuntimeError):
                opts.run_vcpkg_initialized_check(local_vcpkg_only=True)

        out = capsys.readouterr().out
        assert "enable-local-vcpkg" in out
        assert "VCPKG_ROOT" not in out
        assert "--vcpkg-toolchain" not in out
        assert "--use-system-libs" not in out

    def test_run_vcpkg_check_passes_with_system_libs(self, ctx_mock):
        """use_system_libs=True must bypass the toolchain check entirely."""
        opts = ClickContext.BuildOptions(ctx_mock.vcpkg_dir)
        opts.vcpkg_toolchain = None
        opts.use_system_libs = True
        opts.run_vcpkg_initialized_check()

    def test_script_prefix_shown_as_python3_for_py_files(self, ctx_mock, capsys):
        """When invoked as a .py file the guidance must show 'python3 dev.py'."""
        opts = ClickContext.BuildOptions(ctx_mock.vcpkg_dir)
        opts.vcpkg_toolchain = None
        opts.use_system_libs = False

        with patch("sys.argv", ["dev.py"]):
            with pytest.raises(RuntimeError):
                opts.run_vcpkg_initialized_check()

        out = capsys.readouterr().out
        assert "python3 dev.py" in out


class TestTripletDetection:
    @pytest.fixture
    def vcpkg_dir(self):
        return "/mock/vcpkg"

    @pytest.mark.parametrize("machine,system,expected", [
        ("x86_64", "Linux", "x64-linux"),
        ("amd64", "Linux", "x64-linux"),
        ("aarch64", "Linux", "arm64-linux"),
        ("arm64", "Linux", "arm64-linux"),
        ("x86_64", "Darwin", "x64-osx"),
        ("arm64", "Darwin", "arm64-osx"),
        ("x86_64", "Windows", "x64-windows-static-md"),
        ("arm64", "Windows", "arm64-windows-static-md"),
    ])
    def test_triplet_matrix(self, vcpkg_dir, machine, system, expected):
        with patch("platform.machine", return_value=machine), \
             patch("platform.system", return_value=system):
            opts = ClickContext.BuildOptions(vcpkg_dir)
            assert opts.vcpkg_triplet == expected

    def test_unknown_arch_returns_none_triplet(self, vcpkg_dir):
        with patch("platform.machine", return_value="mips"), \
             patch("platform.system", return_value="Linux"):
            opts = ClickContext.BuildOptions(vcpkg_dir)
            assert opts.vcpkg_triplet is None


class TestDevPersistentDir:
    def test_defaults_to_home_osdkv2(self):
        env = {k: v for k, v in os.environ.items() if k != "DEV_PERSISTENT_DIR"}
        with patch.dict(os.environ, env, clear=True), \
             patch("os.makedirs"), patch("os.path.isdir", return_value=True):
            ctx = ClickContext(MagicMock())
        assert ctx._dev_persistent_dir == os.path.join(os.path.expanduser("~"), ".osdkv2")

    def test_env_var_overrides_default(self):
        with patch.dict(os.environ, {"DEV_PERSISTENT_DIR": "/custom/path"}), \
             patch("os.makedirs"), patch("os.path.isdir", return_value=True):
            ctx = ClickContext(MagicMock())
        assert ctx._dev_persistent_dir == "/custom/path"

    def test_env_var_printed_to_stdout(self, capsys):
        with patch.dict(os.environ, {"DEV_PERSISTENT_DIR": "/custom/path"}), \
             patch("os.makedirs"), patch("os.path.isdir", return_value=True):
            ClickContext(MagicMock())
        assert "/custom/path" in capsys.readouterr().out
