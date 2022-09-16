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
        groups = re.search("set\(OusterSDK_VERSION_STRING ([^-\)]+)(.(.*))?\)", content)
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

        if platform.system() == "Windows":
            build_args += ['--', '/m']
        else:
            build_args += ['--', '-j2']

        # pass OUSTER_SDK_PATH to cmake
        cmake_args += ['-DOUSTER_SDK_PATH=' + OUSTER_SDK_PATH]

        # specify additional cmake args
        env = os.environ.copy()
        extra_args = env.get('OUSTER_SDK_CMAKE_ARGS')
        if extra_args:
            cmake_args += shlex.split(extra_args)

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args,
                              cwd=self.build_temp,
                              env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args,
                              cwd=self.build_temp)


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


setup(
    name='ouster-sdk',
    url='https://github.com/ouster-lidar/ouster_example',
    # read from top-level sdk CMakeLists.txt
    version=parse_version(),
    package_dir={'': 'src'},
    packages=find_namespace_packages(where='src', include='ouster.*'),
    package_data={
        'ouster.client': ['py.typed', '_client.pyi'],
        'ouster.pcap': ['py.typed', '_pcap.pyi'],
        'ouster.sdk': ['py.typed', '_viz.pyi'],
    },
    author='Ouster Sensor SDK Developers',
    author_email='oss@ouster.io',
    description='Ouster sensor SDK',
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
    python_requires='>=3.7, <4',
    install_requires=[
        'more-itertools >=8.6',
        'numpy >=1.19, <2, !=1.19.4',
        'typing-extensions >=3.7',
    ],
    extras_require={
        'test': ['pytest >=7.0, <8'],
        'dev': ['flake8', 'mypy', 'pylsp-mypy', 'python-lsp-server', 'yapf'],
        'docs': [
            'Sphinx >=3.5',
            'sphinx-autodoc-typehints ==1.17.0',
            'sphinx-rtd-theme ==1.0.0',
            'sphinx-copybutton ==0.5.0',
            'docutils <0.18',
            'sphinx-tabs ==3.3.1',
            'breathe ==4.33.1'
        ],
        'examples': [
            'matplotlib',
            'opencv-python',
            'laspy',
            'PyQt5; platform_system=="Windows"',
        ],
    },
    entry_points={'console_scripts': ['simple-viz=ouster.sdk.simple_viz:main']})
