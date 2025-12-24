#! /bin/bash

set -ex
currentDir="$(cd "$(dirname "$0")" && pwd)"
baseDir="$currentDir/../.."
tempDir="$(mktemp -d)"
VCPKG_BINARY_SOURCES=${VCPKG_BINARY_SOURCES:-""}
APT_PROXY=${APT_PROXY:-""}

baseImage="ubuntu:22.04"
if ! [ -z "$1" ]; then
  baseImage="$1"
fi

trap 'rm -rf $tempDir' EXIT
trap 'echo \*\*\* ERROR on line: $LINENO exit_code: $?' ERR

cd "$baseDir"
pwd
docker build -f "$currentDir/Dockerfile" --iidfile="$tempDir/iid" \
  --network host \
  --build-arg BASE="$baseImage" \
  --build-arg APT_PROXY="$APT_PROXY" \
  --build-arg VCPKG_BINARY_SOURCES="$VCPKG_BINARY_SOURCES" .

docker run --rm "$(cat "$tempDir/iid")"
