#! /bin/bash

PYTHON_=$(which python3.8 python3.9 python3.10 | head -1)
PYTHON="${DOCKER_PYTHON:-$PYTHON_}"

$PYTHON -m pip install --no-cache-dir --user -U pip pybind11 mypy flake8 flake8-formatter-junit-xml wheel pytest

SCRIPT_DIR=$(cd $(dirname $0) && pwd)
pushd $SCRIPT_DIR
DEPS=$(python3 -- <<EOF
import setup

def format_item(item):
    output = item.replace("\"", "\'")
    output = f"\"{output}\""
    return output

print(" ".join([format_item(x) for x in setup.install_requires()]))
EOF
)
popd
echo $DEPS | xargs $PYTHON -m pip install
