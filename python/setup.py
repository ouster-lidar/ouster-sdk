import os
import sys
import shlex
import shutil
import subprocess
import platform
import glob
import zipfile
import stat
import time
import json
import re
import socket
import datetime

from setuptools import setup, find_namespace_packages, Extension
from setuptools.command.build_ext import build_ext
from setuptools.command.sdist import sdist
from wheel.bdist_wheel import bdist_wheel

SRC_PATH = os.path.dirname(os.path.abspath(__file__))


def _resolve_ouster_sdk_path():
    """Absolute path to ouster-sdk repo root (directory that contains cmake/)."""
    ouster_sdk_path = os.getenv('OUSTER_SDK_PATH')
    if ouster_sdk_path is None:
        ouster_sdk_path = os.path.join(SRC_PATH, "sdk")
    if not os.path.exists(ouster_sdk_path):
        ouster_sdk_path = os.path.dirname(SRC_PATH)
    if not os.path.exists(os.path.join(ouster_sdk_path, "cmake")):
        raise RuntimeError("Could not guess OUSTER_SDK_PATH")
    return os.path.abspath(ouster_sdk_path)


def _is_local_versioning_available(sdk_root):
    vm = os.path.join(
        sdk_root, '_private', 'scripts', 'dev_script_library',
        'dev_version_manager.py')
    return vm if os.path.isfile(vm) else None


def _run_version_sdk_subprocess(cmd, cwd, env):
    # Pass version_manager stderr through so logging shows during `pip install`
    # (otherwise capture_output hides it on success).
    proc = subprocess.run(
        cmd,
        cwd=cwd,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=sys.stderr,
    )
    return proc.returncode, (proc.stdout or "")


def set_local_project_version(log_fn=print):
    """Run dev_version_manager when _private/scripts/dev_script_library is present."""
    try:
        sdk_root = _resolve_ouster_sdk_path()
    except RuntimeError:
        return
    if not _is_local_versioning_available(sdk_root):
        return
    dev_cli = os.path.join(sdk_root, "scripts", "dev.py")
    if not os.path.isfile(dev_cli):
        return
    local_version_filepath = os.path.join(sdk_root, "VERSION.generated")
    branch = os.environ.get("BRANCH_NAME")
    if branch is None and os.path.isfile(local_version_filepath):
        os.remove(local_version_filepath)
    cmd = [
        sys.executable, dev_cli, "utils", "version-sdk",
        "--repo", sdk_root,
    ]
    if branch:
        cmd.extend(["--branch", branch])

    rc, comb = _run_version_sdk_subprocess(cmd, sdk_root, os.environ.copy())
    if rc != 0:
        log_fn(
            "set_local_project_version: version_manager failed; "
            "falling back to repo VERSION files.\n"
            + comb
        )
        return
    if not os.path.isfile(local_version_filepath):
        log_fn(
            "set_local_project_version: version_manager produced no VERSION.generated file"
        )
        return


def get_version():
    def read_version(path):
        version = {}
        with open(path) as f:
            exec(f.read(), version)
        return version["__version__"]

    version_file = os.path.abspath(os.path.join(
        os.path.dirname(__file__),
        "..",
        "VERSION"))
    bdist_file_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__),
        "sdk",
        "VERSION"))
    try:
        sdk_root = _resolve_ouster_sdk_path()
        local_version_filepath = (os.environ.get("OUSTER_SDK_VERSION_FILE")
                                   or os.path.join(sdk_root, "VERSION.generated"))
        if os.path.exists(local_version_filepath):
            return read_version(local_version_filepath)
    except RuntimeError:
        pass
    # set_local_project_version() refreshes repo-root VERSION.generated; else fall back.
    if os.path.exists(version_file):
        return read_version(version_file)
    if os.path.exists(bdist_file_path):
        return read_version(bdist_file_path)
    return None


# https://packaging.python.org/en/latest/guides/single-sourcing-package-version/
def parse_version():
    set_local_project_version()
    python_version = get_version()
    if (python_version):
        return python_version
    else:
        raise RuntimeError("Error: Could not read __version__ from VERSION file.")


class BuildProfiler:
    """Collects per-phase wall-clock timings and writes a JSON artifact to ARTIFACT_DIR."""

    def __init__(self):
        self._timers = {}
        self.phases = {}
        self.node = os.environ.get('NODE_NAME') or socket.gethostname()
        self.python_version = platform.python_version()
        self.build_start = (
            datetime.datetime.now(datetime.timezone.utc)
            .isoformat()
            .replace("+00:00", "Z")
        )

    def start(self, phase):
        self._timers[phase] = time.perf_counter()

    def stop(self, phase):
        if phase in self._timers:
            self.phases[phase] = round(time.perf_counter() - self._timers.pop(phase), 3)

    def record(self, phase, value_s):
        self.phases[phase] = round(float(value_s), 3)

    def ingest_line(self, line):
        """Parse a single line of cmake/build output for profiling markers."""
        # [BUILD_PROFILE] key=value emitted by the build system
        m = re.search(r'\[BUILD_PROFILE\]\s+(\w+)=([\d.]+)', line)
        if m:
            self.record(m.group(1), m.group(2))
            return
        # vcpkg self-reported total (e.g. "vcpkg install completed in 135 s")
        m = re.search(r'vcpkg install completed in\s+([\d.]+)\s*s', line, re.IGNORECASE)
        if m:
            self.record('vcpkg_install_s', m.group(1))

    def write_artifact(self, log_fn=print):
        artifact_dir = os.environ.get('ARTIFACT_DIR', '.')
        try:
            os.makedirs(artifact_dir, exist_ok=True)
        except OSError as e:
            log_fn(f"[BUILD_PROFILE] WARNING: cannot create artifact dir {artifact_dir!r}: {e} — skipping profile write")
            return None
        node_safe = re.sub(r'[^\w.-]', '_', self.node)
        filename = f"build_profile_py{self.python_version}_{node_safe}.json"
        data = {
            "build_start_utc": self.build_start,
            "node": self.node,
            "python_version": self.python_version,
            "phases_s": self.phases,
        }
        path = os.path.join(artifact_dir, filename)
        try:
            with open(path, 'w') as f:
                json.dump(data, f, indent=2)
        except OSError as e:
            log_fn(f"[BUILD_PROFILE] WARNING: cannot write artifact {path!r}: {e} — skipping profile write")
            return None
        log_fn(f"[BUILD_PROFILE] artifact written to {path}")
        return path


class CMakeExtension(Extension):
    def __init__(self, name, builddir, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.builddir = os.path.abspath(builddir)
        os.makedirs(self.builddir, exist_ok=True)
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        _resolve_ouster_sdk_path()  # raises RuntimeError if SDK path can't be found
        self._env = os.environ.copy()
        self._build_type = os.getenv("BUILD_TYPE", "Release")
        self._jobs = os.getenv('OUSTER_SDK_BUILD_JOBS', os.cpu_count())
        self._cmake_path = None
        self._profiler = BuildProfiler()
        self.distribution._build_profiler = self._profiler

        set_local_project_version(log_fn=self.cmake_log)

        self.cmake_log(f"CMake: Env: {str(self._env)}")
        self.cmake_log(f"CMake: Build Type: {self._build_type}")
        self.cmake_log(f"CMake: Jobs: {self._jobs}")

        for ext in self.extensions:
            self.install_deps(ext)
            self.cmake_log(f"CMake: Extension: {ext.name} Build Dir: {ext.builddir}")
            self.build_extension(ext)

    def cmake_log(self, message):
        print(message)
        log_file = self._env.get('OUSTER_SDK_CMAKE_LOG_FILE')
        if log_file:
            if os.path.exists(os.path.dirname(log_file)):
                with open(log_file, 'a') as f:
                    f.write(str(message))

    def install_deps(self, ext):
        self._profiler.start('install_deps_s')
        try:
            self.cmake_log(f"CMake: Extension: {ext.name} Install Deps: Download")
            output1 = subprocess.run([sys.executable, "-m", "pip", "download",
                                      "cmake==3.24.2", "nanobind==2.9.2"],
                                     cwd=ext.builddir,
                                     stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT,
                                     env=self._env, text=True)
            self.cmake_log(output1.stdout)
            if output1.returncode != 0:
                self.cmake_log(f"CMake: Extension: {ext.name} Install Deps: Download: Error: "
                               "Failed to download dependencies")
                sys.exit(1)
            self.cmake_log(f"CMake: Extension: {ext.name} Install Deps: Extraction")
            if not os.path.exists(os.path.join(ext.builddir, "cmake")):
                for item in glob.glob(os.path.join(ext.builddir, "*cmake*.whl")):
                    with zipfile.ZipFile(item, 'r') as z:
                        z.extractall(ext.builddir)
            if not os.path.exists(os.path.join(ext.builddir, "pybind11")):
                for item in glob.glob(os.path.join(ext.builddir, "*pybind11*.whl")):
                    with zipfile.ZipFile(item, 'r') as z:
                        z.extractall(ext.builddir)
            if platform.system() in ['Linux', 'Darwin']:
                self._cmake_path = os.path.join(ext.builddir, "cmake", "data", "bin", "cmake")
                perm = os.stat(self._cmake_path).st_mode
                os.chmod(self._cmake_path, perm | stat.S_IXUSR)
            else:
                self._cmake_path = os.path.join(ext.builddir, "cmake", "data", "bin", "cmake.exe")
        finally:
            self._profiler.stop('install_deps_s')

    def cmake_config(self, ext):
        extdir = os.path.abspath(
            os.path.dirname(self.get_ext_fullpath(ext.name)))
        # required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        ouster_sdk_path = _resolve_ouster_sdk_path()
        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DPython_ROOT_DIR={sys.base_prefix}",
            '-DBUILD_SHARED_LIBS:BOOL=OFF',
            f"-DCMAKE_BUILD_TYPE={self._build_type}",
            f"-DOUSTER_SDK_PATH={ouster_sdk_path}"
        ]
        if "VCPKG_MANIFEST_MODE" in self._env:
            cmake_args.append(f"-DVCPKG_MANIFEST_MODE={self._env['VCPKG_MANIFEST_MODE']}")
        else:
            cmake_args.append("-DVCPKG_MANIFEST_MODE=ON")
        if "CMAKE_TOOLCHAIN_FILE" in self._env:
            cmake_args.append(f"-DCMAKE_TOOLCHAIN_FILE={self._env['CMAKE_TOOLCHAIN_FILE']}")
        if "VCPKG_TARGET_TRIPLET" in self._env:
            cmake_args.append(f"-DVCPKG_TARGET_TRIPLET={self._env['VCPKG_TARGET_TRIPLET']}")
        if "VCPKG_MAX_CONCURRENCY" in self._env:
            cmake_args.append(f"-DVCPKG_MAX_CONCURRENCY={self._env['VCPKG_MAX_CONCURRENCY']}")
        if "VCPKG_MANIFEST_DIR" in self._env:
            cmake_args.append(f"-DVCPKG_MANIFEST_DIR={self._env['VCPKG_MANIFEST_DIR']}")
        else:
            cmake_args.append(f"-DVCPKG_MANIFEST_DIR={ouster_sdk_path}")
        if "VCPKG_BINARY_SOURCES" in self._env:
            cmake_args.append(f"-DVCPKG_BINARY_SOURCES={self._env['VCPKG_BINARY_SOURCES']}")
        if "CMAKE_CXX_COMPILER" in self._env:
            cmake_args.append(f"-DCMAKE_CXX_COMPILER={self._env['CMAKE_CXX_COMPILER']}")
        if "CMAKE_C_COMPILER" in self._env:
            cmake_args.append(f"-DCMAKE_C_COMPILER={self._env['CMAKE_C_COMPILER']}")
        if "CMAKE_CXX_FLAGS" in self._env:
            cmake_args.append(f"-DCMAKE_CXX_FLAGS={self._env['CMAKE_CXX_FLAGS']}")
        if "CMAKE_C_FLAGS" in self._env:
            cmake_args.append(f"-DCMAKE_C_FLAGS={self._env['CMAKE_C_FLAGS']}")
        if "USE_OPENMP" in self._env:
            cmake_args.append(f"-DUSE_OPENMP={self._env['USE_OPENMP']}")
        if "OUSTER_LIBRARY_CLEANUP" in self._env:
            cmake_args.append(f"-DOUSTER_LIBRARY_CLEANUP={self._env['OUSTER_LIBRARY_CLEANUP']}")
        if "OUSTER_LIBRARY_CLEANUP_VERIFY" in self._env:
            cmake_args.append(f"-DOUSTER_LIBRARY_CLEANUP_VERIFY={self._env['OUSTER_LIBRARY_CLEANUP_VERIFY']}")

        # specify additional cmake args
        extra_args = self._env.get('OUSTER_SDK_CMAKE_ARGS')
        if extra_args:
            cmake_args += shlex.split(extra_args)
        run = []
        if platform.system() == "Darwin":
            run.extend(["arch", "-arch", platform.machine()])
        run.extend([self._cmake_path, ext.sourcedir])
        run.extend(cmake_args)
        self.cmake_log(f"CMake: Extension: {ext.name} Config: {run}")
        self.cmake_log(f"CMake: Extension: {ext.name} Config: Output:")
        self._profiler.start('cmake_configure_s')
        proc = subprocess.Popen(run, cwd=ext.builddir,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                env=self._env, text=True)
        for line in proc.stdout:
            line = line.rstrip('\n')
            self.cmake_log(line)
            self._profiler.ingest_line(line)
            if re.search(r'Running vcpkg install', line, re.IGNORECASE):
                self._profiler.start('vcpkg_install_s')
            elif 'vcpkg_install_s' in self._profiler._timers and \
                    re.search(r'vcpkg install completed', line, re.IGNORECASE):
                self._profiler.stop('vcpkg_install_s')
        proc.wait()
        self._profiler.stop('cmake_configure_s')
        if proc.returncode != 0:
            self.cmake_log(f"CMake: Extension: {ext.name} Config: Error: Failed to config")
            sys.exit(1)

    def cmake_build(self, ext):
        run = [self._cmake_path, '--build', '.', '--parallel', str(self._jobs), '--config', self._build_type]
        self.cmake_log(f"CMake: Extension: {ext.name} Build: {run}")
        self.cmake_log(f"CMake: Extension: {ext.name} Build: Output:")
        self._profiler.start('cmake_build_s')
        proc = subprocess.Popen(run, cwd=ext.builddir,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                env=self._env, text=True)
        for line in proc.stdout:
            line = line.rstrip('\n')
            self.cmake_log(line)
            self._profiler.ingest_line(line)
        proc.wait()
        self._profiler.stop('cmake_build_s')
        if "OUSTER_BUILD_DIR_COPY" in self._env:
            shutil.copytree(ext.builddir, self._env["OUSTER_BUILD_DIR_COPY"], dirs_exist_ok=True)
        if proc.returncode != 0:
            self.cmake_log(f"CMake: Extension: {ext.name} Build: Error: Failed to build")
            sys.exit(1)

    def _cmake_cache_source_dir(self, ext):
        """Return the source directory recorded in CMakeCache.txt, or None."""
        cache_file = os.path.join(ext.builddir, "CMakeCache.txt")
        try:
            with open(cache_file) as f:
                for line in f:
                    if line.startswith("CMAKE_HOME_DIRECTORY:INTERNAL="):
                        return line.split("=", 1)[1].strip()
        except OSError:
            pass
        return None

    def build_extension(self, ext):
        cache_file = os.path.join(ext.builddir, "CMakeCache.txt")
        cached_src = self._cmake_cache_source_dir(ext)
        cache_valid = (os.path.exists(cache_file)
                       and cached_src is not None
                       and os.path.exists(cached_src))
        if not cache_valid:
            if os.path.exists(cache_file):
                self.cmake_log(f"CMake: Extension: {ext.name} Config: "
                               f"CMake Cache source dir '{cached_src}' no longer exists, reconfiguring")
                os.remove(cache_file)
            else:
                self.cmake_log(f"CMake: Extension: {ext.name} Config: CMake Cache does not exist, running cmake config")
            self.cmake_config(ext)
        else:
            self.cmake_log(f"CMake: Extension: {ext.name} Config: CMake Cache Exists, trying to use cache")

        self.cmake_build(ext)
        # C++ build is done; start timing the wheel packaging phase that bdist_wheel
        # performs after build_ext returns (file finalization, wheel creation, etc.)
        self._profiler.start('wheel_packaging_s')


class sdk_sdist(sdist):
    """Allow including files from parent directory via symlink."""
    def run(self):
        created = False
        try:
            if not os.path.exists("sdk"):
                os.symlink("..", "sdk")
                created = True
            super().run()
        finally:
            if created:
                os.remove("sdk")


def _freeware_artifacts(sdk_root):
    """Return freeware EULA paths when both required _private files are present."""
    base = os.environ.get("OUSTER_SDK_FREEWARE_DIR") or os.path.join(sdk_root, "_private")
    license_path = os.path.join(base, "LICENSE-freeware")
    copyright_path = os.path.join(base, "COPYRIGHT-freeware")
    if os.path.isfile(license_path) and os.path.isfile(copyright_path):
        return license_path, copyright_path
    return None, None


class sdk_bdist_wheel(bdist_wheel):
    """Copy files needed by wheel from SDK dir."""

    FILES = ["LICENSE", "LICENSE-bin", "COPYRIGHT"]

    def run(self):
        staged = []
        try:
            sdk_root = _resolve_ouster_sdk_path()
            version_file = (
                "VERSION.generated"
                if os.path.isfile(os.path.join(sdk_root, "VERSION.generated"))
                else "VERSION"
            )
            shutil.copy(os.path.join(sdk_root, version_file), version_file)
            staged.append(version_file)

            freeware_license, freeware_copyright = _freeware_artifacts(sdk_root)
            has_freeware = freeware_license is not None
            license_files = []
            for name in self.FILES:
                shutil.copy(os.path.join(sdk_root, name), name)
                if name == "COPYRIGHT" and has_freeware:
                    with open("COPYRIGHT", "a", encoding="utf-8") as out:
                        with open(freeware_copyright, encoding="utf-8") as extra:
                            out.write("\n\n")
                            out.write(extra.read())
                staged.append(name)
                license_files.append(name)

            if has_freeware:
                shutil.copy(freeware_license, "LICENSE-freeware")
                staged.append("LICENSE-freeware")
                license_files.insert(0, "LICENSE-freeware")

            self.distribution.metadata.license_files = license_files
            if hasattr(self.distribution, "_finalize_license_files"):
                self.distribution._finalize_license_files()

            # super().run() triggers build_ext → CMakeBuild.run() which creates
            # the profiler and attaches it to self.distribution._build_profiler
            super().run()
        finally:
            for file in staged:
                if os.path.exists(file):
                    os.remove(file)
            # Profiler is only available after super().run() has completed
            profiler = getattr(self.distribution, '_build_profiler', None)
            if profiler:
                profiler.stop('wheel_packaging_s')
                profiler.write_artifact(log_fn=print)


def install_requires():
    install_requires = []
    with open(os.path.join(SRC_PATH, "requirements.txt")) as f:
        for line in f:
            install_requires.append(line)
    return install_requires


if __name__ == "__main__":
    OUSTER_SDK_PATH = _resolve_ouster_sdk_path()
    setup(
        name='ouster_sdk',
        url='https://github.com/ouster-lidar/ouster-sdk',
        version=parse_version(),
        package_dir={'': 'src'},
        packages=find_namespace_packages(where='src', include='ouster.*'),
        package_data={
            'ouster.sdk.client': ['py.typed', '_client.pyi'],
            'ouster.sdk.pcap': ['py.typed', '_pcap.pyi'],
            'ouster.sdk.osf': ['py.typed', '_osf.pyi'],
            'ouster.sdk.viz': ['py.typed', '_viz.pyi'],
            'ouster.sdk.algorithm': ['py.typed'],
            'ouster.sdk.mapping': ['py.typed', '_mapping.pyi'],
            'ouster.sdk.bag': ['py.typed'],
            'ouster.cli': ['sensor_replay_dockerfile', 'templates/*.html']
        },
        include_package_data=True,
        author='Ouster Sensor SDK Developers',
        author_email='oss@ouster.io',
        description='Ouster Sensor SDK',
        license='BSD-3-Clause AND Ouster-Freeware-EULA (See project links)',
        ext_modules=[
            CMakeExtension('ouster.*',
                           os.path.join(OUSTER_SDK_PATH, "build", f"py{platform.python_version()}"))
        ],
        cmdclass={
            'build_ext': CMakeBuild,
            'sdist': sdk_sdist,
            'bdist_wheel': sdk_bdist_wheel,
        },
        zip_safe=False,
        python_requires='>=3.8, <4',
        install_requires=install_requires(),
        extras_require={
            'test': [
                'pytest >=7.0, <8',
                'pytest-asyncio',
                'iniconfig <=2.1.0',
            ],
            'dev': ['flake8', 'mypy', 'pylsp-mypy', 'python-lsp-server', 'yapf', "nanobind==2.9.2", "numpy"],
            'docs': [
                'Sphinx >=3.5',
                'sphinx-autodoc-typehints ==1.17.0',
                'sphinx-rtd-theme ==1.0.0',
                'sphinx-copybutton ==0.5.0',
                'docutils <0.18',
                'sphinx-tabs ==3.3.1',
                'breathe ==4.33.1',
                'sphinx-rtd-size'
            ],
            'examples': [
                'matplotlib',
                'opencv-python',
                'laspy',
                'PyQt5; platform_system=="Windows"',
            ],
            'cleanup': [
                'pydemumble',
                'tree-sitter-cpp',
                'tree-sitter',
                'libclang',
            ],
        },
        entry_points={'console_scripts':
            [
                'simple-viz=ouster.sdk.simple_viz:main',                     # TODO[UN]: do we need to keep?
                'ouster-cli=ouster.cli.core:run'
            ]
        }
    )
