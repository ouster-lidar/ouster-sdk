import os
import sys
import shlex
import shutil
import subprocess
import platform
import glob
import zipfile
import stat

from setuptools import setup, find_namespace_packages, Extension
from setuptools.command.build_ext import build_ext
from setuptools.command.sdist import sdist
from wheel.bdist_wheel import bdist_wheel

SRC_PATH = os.path.dirname(os.path.abspath(__file__))
if __name__ == "__main__":
    # use SDK source location from environment or try to guess
    OUSTER_SDK_PATH = os.getenv('OUSTER_SDK_PATH')
    if OUSTER_SDK_PATH is None:
        OUSTER_SDK_PATH = os.path.join(SRC_PATH, "sdk")
    if not os.path.exists(OUSTER_SDK_PATH):
        OUSTER_SDK_PATH = os.path.dirname(SRC_PATH)
    if not os.path.exists(os.path.join(OUSTER_SDK_PATH, "cmake")):
        raise RuntimeError("Could not guess OUSTER_SDK_PATH")


def get_version():
    version_file = os.path.abspath(os.path.join(
                                os.path.dirname(__file__),
                                "..",
                                "VERSION"))
    bdist_file_path = os.path.abspath(os.path.join(
                                os.path.dirname(__file__),
                                "sdk",
                                "VERSION"))
    if os.path.exists(bdist_file_path):
        version = {}
        with open(bdist_file_path) as f:
            exec(f.read(), version)
        return version["__version__"]
    if os.path.exists(version_file):
        version = {}
        with open(version_file) as f:
            exec(f.read(), version)
        return version["__version__"]
    return None


# https://packaging.python.org/en/latest/guides/single-sourcing-package-version/
def parse_version():
    python_version = get_version()
    if (python_version):
        return python_version
    else:
        raise RuntimeError("Error: Could not read __version__ from VERSION file.")


class CMakeExtension(Extension):
    def __init__(self, name, builddir, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.builddir = os.path.abspath(builddir)
        os.makedirs(self.builddir, exist_ok=True)
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        self._env = os.environ.copy()
        # Bug in pybind11 cmake strips symbols with RelWithDebInfo
        # https://github.com/pybind/pybind11/issues/1891
        # Fixed in https://github.com/pybind/pybind11/issues/1892
        self._build_type = os.getenv("BUILD_TYPE", "Release")
        self._jobs = os.getenv('OUSTER_SDK_BUILD_JOBS', os.cpu_count())
        self._cmake_path = None

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
        self.cmake_log(f"CMake: Extension: {ext.name} Install Deps: Download")
        output1 = subprocess.run([sys.executable, "-m", "pip", "download", "pybind11==2.12.0", "cmake==3.24.2"],
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

    def cmake_config(self, ext):
        extdir = os.path.abspath(
            os.path.dirname(self.get_ext_fullpath(ext.name)))
        # required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            '-DBUILD_SHARED_LIBS:BOOL=OFF',
            f"-DCMAKE_BUILD_TYPE={self._build_type}",
            f"-DOUSTER_SDK_PATH={OUSTER_SDK_PATH}"
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
            cmake_args.append(f"-DVCPKG_MANIFEST_DIR={OUSTER_SDK_PATH}")
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
        output1 = subprocess.run(run,
                                 cwd=ext.builddir,
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT,
                                 env=self._env, text=True)
        self.cmake_log(f"CMake: Extension: {ext.name} Config: Output:")
        self.cmake_log(output1.stdout)
        if output1.returncode != 0:
            self.cmake_log(f"CMake: Extension: {ext.name} Config: Error: Failed to config")
            sys.exit(1)

    def cmake_build(self, ext):
        run = [self._cmake_path, '--build', '.', '--parallel', str(self._jobs), '--config', self._build_type]
        self.cmake_log(f"CMake: Extension: {ext.name} Build: {run}")
        output2 = subprocess.run(run,
                                 cwd=ext.builddir,
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT,
                                 env=self._env, text=True)
        self.cmake_log(f"CMake: Extension: {ext.name} Build: Output:")
        self.cmake_log(output2.stdout)
        if "OUSTER_BUILD_DIR_COPY" in self._env:
            shutil.copytree(ext.builddir, self._env["OUSTER_BUILD_DIR_COPY"], dirs_exist_ok=True)
        if output2.returncode != 0:
            self.cmake_log(f"CMake: Extension: {ext.name} Build: Error: Failed to build")
            sys.exit(1)

    def build_extension(self, ext):
        if not os.path.exists(os.path.join(ext.builddir, "CMakeCache.txt")):
            self.cmake_log(f"CMake: Extension: {ext.name} Config: CMake Cache does not exist, running cmake config")
            self.cmake_config(ext)
        else:
            self.cmake_log(f"CMake: Extension: {ext.name} Config: CMake Cache Exists, trying to use cache")

        self.cmake_build(ext)


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


class sdk_bdist_wheel(bdist_wheel):
    """Copy files needed by wheel from SDK dir."""

    FILES = ["LICENSE", "THIRD_PARTY_NOTICES", "COPYRIGHT", "VERSION"]

    def run(self):
        try:
            for file in self.FILES:
                shutil.copy(os.path.join(OUSTER_SDK_PATH, file), ".")
            super().run()
        finally:
            for file in self.FILES:
                if os.path.exists(file):
                    os.remove(file)


def install_requires():
    install_requires = []
    with open(os.path.join(SRC_PATH, "requirements.txt")) as f:
        for line in f:
            install_requires.append(line)
    return install_requires


if __name__ == "__main__":
    setup(
        name='ouster_sdk',
        url='https://github.com/ouster-lidar/ouster-sdk',
        # read from top-level sdk CMakeLists.txt
        version=parse_version(),
        package_dir={'': 'src'},
        packages=find_namespace_packages(where='src', include='ouster.*'),
        package_data={
            'ouster.sdk.client': ['py.typed', '_client.pyi'],
            'ouster.sdk.pcap': ['py.typed', '_pcap.pyi'],
            'ouster.sdk.osf': ['py.typed', '_osf.pyi'],
            'ouster.sdk.viz': ['py.typed', '_viz.pyi'],
            'ouster.sdk.mapping': ['py.typed', '_mapping.pyi'],
            'ouster.sdk.bag': ['py.typed'],
            'ouster.cli': ['sensor_replay_dockerfile', 'templates/*.html']
        },
        include_package_data=True,
        author='Ouster Sensor SDK Developers',
        author_email='oss@ouster.io',
        description='Ouster Sensor SDK',
        license='BSD 3-Clause License',
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
            'dev': ['flake8', 'mypy', 'pylsp-mypy', 'python-lsp-server', 'yapf'],
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
        },
        entry_points={'console_scripts':
            [
                'simple-viz=ouster.sdk.simple_viz:main',                     # TODO[UN]: do we need to keep?
                'ouster-cli=ouster.cli.core:run'
            ]
        }
    )
