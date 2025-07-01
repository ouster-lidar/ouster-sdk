#!/bin/bash

set -euo pipefail

SOURCE_DIR=$(cd $(dirname $0)/.. && pwd)
CWD=$(pwd)

help='false'
apply='false'
check='true'
SUBCOMMAND='--Werror --dry-run'
#Input arguments
while getopts 'ha' flag
do
    case "${flag}" in
        h) help='true';;
        a) apply='true';;
    esac
done

if [[ $help == 'true' ]]; then
  echo """
  How to run:
  cd ouster-sdk; then run ./scripts/clang-linting.sh
  Default: performs a clang format check and prints errors if they exist
  ./scripts/clang-linting.sh -a
  applies the clang format to the files
  """
fi

if [[ $apply == 'true' ]]; then
  check='false'
  SUBCOMMAND=''
fi

# Ignore the ./ouster_client/include/optional-lite/nonstd/optional.hpp file while formatting files
run_command() {
    cd $SOURCE_DIR
    find . -regex '.*\.\(cpp\|hpp\|cu\|c\|h\)' -not -path './ouster_client/include/optional-lite/nonstd/optional.hpp' \
         -and -not -path './build/*' \
         -and -not -path './thirdparty/*' \
         -and -not -path './sdk-extensions/*' \
         -exec clang-format ${SUBCOMMAND} -style=file -i {} \;
    cd $CWD
    echo "Exit code: $?"
}

# grep returns an exit code of 1 if it finds no matches.
set +e
OUTPUT=$(run_command 2>&1 | grep error)
set -e
echo "OUTPUT $OUTPUT"

if [ -z "$OUTPUT" ]; then
  echo "clang-format check passed! No errors"
  exit 0;
else
  run_command
  exit 1;
fi
