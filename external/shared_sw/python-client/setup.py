import os
import re
import sys
import platform
import subprocess

from setuptools import setup, find_namespace_packages, Extension
from setuptools.command.build_ext import build_ext


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

        cfg = 'RelWithDebugInfo' if self.debug else 'Release'
        build_args = ['--config', cfg]

        if platform.system() == "Windows":
            cmake_args += [
                '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(
                    cfg.upper(), extdir)
            ]
            if sys.maxsize > 2**32:
                cmake_args += ['-A', 'x64']
            build_args += ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
            build_args += ['--', '-j2']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(
            env.get('CXXFLAGS', ''), self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args,
                              cwd=self.build_temp,
                              env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args,
                              cwd=self.build_temp)


setup(
    name='ouster-client',
    url='https://bitbucket.org/ouster_io/shared_sw',
    version='0.0.2-dev',
    package_dir={'': 'src'},
    packages=find_namespace_packages(where='src'),
    package_data={'ouster.client': ['py.typed']},
    author='Dima Garbuzov, Chris Bayruns',
    author_email='dima.garbuzov@ouster.io, chris.bayruns@ouster.io',
    description='Ouster sensor client python bindings',
    long_description='',
    ext_modules=[CMakeExtension('ouster.client._sensor')],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
    python_requires='>=3.6',
    install_requires=[
        'numpy',
        'dataclasses >= 0.7; python_version < "3.7"',
    ],
    extras_require={
        'test': ['tox']
    }
)
