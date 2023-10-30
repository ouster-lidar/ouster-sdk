#! /bin/bash

set -e
currentDir="$(cd $(dirname $0) && pwd)"
baseDir=$currentDir/../..
tempDir="$(mktemp -d)"

trap 'rm -rf $tempDir' EXIT
trap 'echo \*\*\* ERROR on line: $LINENO exit_code: $?' ERR

cd $baseDir

docker build -f $currentDir/Dockerfile --iidfile=$tempDir/iid .

docker run $(cat $tempDir/iid)
