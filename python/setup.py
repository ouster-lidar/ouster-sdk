import os
import re
import sys
import platform
import shlex
import shutil
import subprocess

from setuptools import setup, find_namespace_packages, Extension
from setuptools.command.build_ext import build_ext
from setuptools.command.sdist import sdist
from wheel.bdist_wheel import bdist_wheel

# use SDK source location from environment or try to guess
SRC_PATH = os.path.dirname(os.path.abspath(__file__))
OUSTER_SDK_PATH = os.getenv('OUSTER_SDK_PATH')
if OUSTER_SDK_PATH is None:
    OUSTER_SDK_PATH = os.path.join(SRC_PATH, "sdk")
if not os.path.exists(OUSTER_SDK_PATH):
    OUSTER_SDK_PATH = os.path.dirname(SRC_PATH)
if not os.path.exists(os.path.join(OUSTER_SDK_PATH, "cmake")):
    raise RuntimeError("Could not guess OUSTER_SDK_PATH")


# https://packaging.python.org/en/latest/guides/single-sourcing-package-version/
def parse_version():
    with open(os.path.join(OUSTER_SDK_PATH, 'CMakeLists.txt')) as listfile:
        content = listfile.read()
        groups = re.search(r"set\(OusterSDK_VERSION_STRING ([^-\)]+)(.(.*))?\)", content)
        return groups.group(1) + (groups.group(3) or "")


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        import shutil

        if shutil.which('cmake') is None:
            raise RuntimeError("No cmake executable found on path")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(
            os.path.dirname(self.get_ext_fullpath(ext.name)))
        # required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        cmake_args = [
            '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
            '-DPYTHON_EXECUTABLE=' + sys.executable
        ]

        # Bug in pybind11 cmake strips symbols with RelWithDebInfo
        # https://github.com/pybind/pybind11/issues/1891
        cfg = 'Debug' if self.debug else 'Release'

        cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
        build_args = ['--config', cfg]

        env = os.environ.copy()
        jobs = os.getenv('OUSTER_SDK_BUILD_JOBS', os.cpu_count())
        build_args += ['--', f'-j{jobs}']

        if platform.system() == "Windows":
            cmake_args += ['-GNinja']

        # pass OUSTER_SDK_PATH to cmake
        cmake_args += ['-DOUSTER_SDK_PATH=' + OUSTER_SDK_PATH]

        # specify additional cmake args
        extra_args = env.get('OUSTER_SDK_CMAKE_ARGS')
        if extra_args:
            cmake_args += shlex.split(extra_args)
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        def cmake_log(message):
            env = os.environ.copy()
            log_file = env.get('OUSTER_SDK_CMAKE_LOG_FILE')
            if log_file:
                if os.path.exists(os.path.dirname(log_file)):
                    with open(log_file, 'a') as f:
                        f.write(str(message))
            print(message)

        cmake_log("Running: ")
        run = ['cmake', ext.sourcedir] + cmake_args
        cmake_log(run)
        output1 = subprocess.run(run,
                                 cwd=self.build_temp,
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT,
                                 env=env, text=True)
        cmake_log("CMAKE CONFIG OUTPUT")
        cmake_log(output1.stdout)
        if output1.returncode != 0:
            raise "Error running cmake"

        output2 = subprocess.run(['cmake', '--build', '.'] + build_args,
                                 cwd=self.build_temp,
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT,
                                 env=env, text=True)
        cmake_log("CMAKE BUILD OUTPUT")
        cmake_log(output2.stdout)
        if output2.returncode != 0:
            raise "Error running cmake --build"


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

    FILES = ["LICENSE", "LICENSE-bin"]

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
    install_requires = [
        'psutil >=5.9.5, <6',
        'zeroconf >=0.131.0',
        'click >=8.1.3, <9',
        'importlib_metadata ==6.6.0',
        'prettytable >= 2.1.0',
        'requests >=2.0, <3',
        'more-itertools >=8.6',
        'numpy >=1.19, <2, !=1.19.4',
        # scipy is not supported on Mac M1 with Mac OS < 12.0
        'scipy >=1.7, <2;platform_system != "Darwin" or platform_machine != "arm64" or platform_version >= "21.0.0"',
        'typing-extensions >=3.7.4.3',
        'Pillow >=9.2',
        'packaging',
        'setuptools; python_version >= "3.12"',
    ]
    env = os.environ.copy()
    skip_mapping = env.get('OUSTER_SDK_SKIP_MAPPING')
    if not skip_mapping:
        install_requires.append('ouster-mapping==0.2.1; python_version >= "3.8" and python_version <= "3.12"')

    return install_requires


setup(
    name='ouster-sdk',
    url='https://github.com/ouster-lidar/ouster_example',
    # read from top-level sdk CMakeLists.txt
    version=parse_version(),
    package_dir={'': 'src'},
    packages=find_namespace_packages(where='src', include='ouster.*'),
    package_data={
        'ouster.sdk.client': ['py.typed', '_client.pyi'],
        'ouster.sdk.pcap': ['py.typed', '_pcap.pyi'],
        'ouster.sdk.osf': ['py.typed', '_osf.pyi'],
        'ouster.sdk.viz': ['py.typed', '_viz.pyi'],
        'ouster.sdk.bag': ['py.typed']
    },
    author='Ouster Sensor SDK Developers',
    author_email='oss@ouster.io',
    description='Ouster Sensor SDK',
    license='BSD 3-Clause License',
    ext_modules=[
        CMakeExtension('ouster.*'),
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
            'flask==2.2.5'
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
            'simple-viz=ouster.sdk.simple_viz:main',                        # TODO[UN]: do we need to keep?
            'convert-meta-to-legacy=ouster.sdk.convert_to_legacy:main',     # TODO[UN]: do we need to keep?
            'ouster-cli=ouster.cli.core:run'
        ]
    }
)
