#!/bin/bash

set -e

echo =========================================
echo === Welcome to Ouster SDK test gizmo! ===
echo =========================================

# get distro ids, used in artifact paths to generate in tox.ini
source /etc/os-release
export ID VERSION_ID

# do all python work in venv provisioned during build stage
source VENV/bin/activate

# workaround to allow mounting source in read-only mode
# default OUSTER_SDK_PATH will be sym linked to writable place `opt/ws/ouster_sdk_build`
# and updated OUSTER_SDK_PATH will be reexported to env so tox CMD still works
# as expected.
echo
echo "NOTE: sym-linked OUSTER_SDK_PATH to './ouster_sdk_build' location."
echo "      Use './ouster_sdk_build' as a base path inside a container"
echo "      to have a writable source tree (in the case of :ro volume mount)."
echo

mkdir -p ${WORKDIR}/ouster_sdk_build
cp -rus ${OUSTER_SDK_PATH}/* ${WORKDIR}/ouster_sdk_build
export OUSTER_SDK_PATH="${WORKDIR}/ouster_sdk_build"

echo "==== ENV VARS: =============="
printenv
echo

exec "$@"

