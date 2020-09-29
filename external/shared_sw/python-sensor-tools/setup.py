from setuptools import setup, find_namespace_packages

setup(
    name='ouster-sensor-tools',
    url='https://bitbucket.org/ouster_io/ouster_sw',
    version='0.0.2-dev',
    package_dir={'': 'src'},
    packages=find_namespace_packages(where='src'),
    package_data={'ouster.cli': ['py.typed']},
    author='Dima Garbuzov',
    author_email='dima.garbuzov@ouster.io',
    description='Ouster internal sensor utilities',
    long_description='',
    python_requires='>=3.6, <4',
    install_requires=[
        'click >=7, <8',
        'numpy >=1.19, <2',
        'ouster-client ==0.0.2-dev',
        'psutil >=5.7, <6'
    ],
    extras_require={
        'test': [
            'pytest',
            'tox'
        ],
        'dev': [
            'flake8',
            'future',
            'mypy',
            'pyls-mypy',
            'python-language-server',
            'yapf'
        ]
    },
    entry_points={
        'console_scripts': [
            'ouster-checker=ouster.cli.checker:run',
            'ouster-example=ouster.cli.example:run',
            'ouster-cli=ouster.cli.core:run'
        ]
    }
)
