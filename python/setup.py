import os
import sys
import platform
import subprocess

from setuptools import setup, find_namespace_packages, Extension
from setuptools.command.build_ext import build_ext
from setuptools.command.sdist import sdist


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
        build_args = ['--config', cfg]

        if platform.system() == "Windows":
            if sys.maxsize > 2**32:
                cmake_args += ['-A', 'x64']
            build_args += ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
            build_args += ['--', '-j2']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(
            env.get('CXXFLAGS', ''), self.distribution.get_version())

        # allow specifying toolchain in env
        toolchain = env.get('CMAKE_TOOLCHAIN_FILE')
        if toolchain:
            cmake_args += ['-DCMAKE_TOOLCHAIN_FILE=' + toolchain]

        # specify VCPKG triplet in env
        triplet = env.get('VCPKG_TARGET_TRIPLET')
        if triplet:
            cmake_args += ['-DVCPKG_TARGET_TRIPLET=' + triplet]

        # use sdk path from env or location in sdist
        sdk_path = env.get('OUSTER_SDK_PATH')
        sdist_sdk_path = os.path.join(ext.sourcedir, "sdk")
        if sdk_path:
            cmake_args += ['-DOUSTER_SDK_PATH=' + sdk_path]
        elif os.path.exists(sdist_sdk_path):
            cmake_args += ['-DOUSTER_SDK_PATH=' + sdist_sdk_path]

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args,
                              cwd=self.build_temp,
                              env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args,
                              cwd=self.build_temp)


# allow including files from parent directory via symlink
class SDKDist(sdist):
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


setup(
    name='ouster-sdk',
    url='https://github.com/ouster-lidar/ouster_example',
    version='0.2.1',
    package_dir={'': 'src'},
    packages=find_namespace_packages(where='src'),
    namespace_packages=['ouster'],
    package_data={
        'ouster.client': ['py.typed'],
        'ouster.pcap': ['py.typed'],
    },
    author='Ouster SW Developers',
    description='Ouster sensor SDK',
    license='BSD 3-Clause License',
    ext_modules=[
        CMakeExtension('ouster.*'),
    ],
    cmdclass={
        'build_ext': CMakeBuild,
        'sdist': SDKDist,
    },
    zip_safe=False,
    python_requires='>=3.6, <4',
    install_requires=[
        'dataclasses >=0.7; python_version >="3.6" and python_version <"3.7"',
        'more-itertools >=8.6',
        'numpy >=1.19, <2, !=1.19.4',
        'typing-extensions >=3.7',
    ],
    extras_require={
        'test': ['pytest', 'tox'],
        'dev': [
            'flake8', 'future', 'mypy', 'mypy-ls', 'python-lsp-server', 'yapf'
        ],
        'docs': [
            'Sphinx >=3.5',
            'sphinx-autodoc-typehints ==1.11.1',
            'sphinx-rtd-theme ==0.5.2',
            'sphinx-copybutton ==0.3.1',
            'docutils <0.17',
            'sphinx-tabs ==3.0.0',
            'open3d',
        ],
        'examples': [
            'matplotlib',
            'opencv-python',
            'PyQt5; platform_system=="Windows"',
        ],
    })
