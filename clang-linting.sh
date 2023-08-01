#!/bin/bash
# How to run:
# cd ouster-sdk; then run ./clang-linting.sh
# Default: performs a clang format check and prints errors if they exist
# ./clang-linting.sh -a
# applies the clang format to the files
apply='false'
check='true'
SUBCOMMAND='--Werror --dry-run'
#Input arguments
while getopts 'a' flag
do
    case "${flag}" in
        a) apply='true';;
    esac
done


if [[ $apply == 'true' ]]; then
  check='false'
  SUBCOMMAND=''
fi


# Ignore the ./ouster_client/include/optional-lite/nonstd/optional.hpp file while formatting files
CMD="find . -regex '.*\.\(cpp\|hpp\|cu\|c\|h\)' -not -path ./ouster_client/include/optional-lite/nonstd/optional.hpp \
-exec clang-format ${SUBCOMMAND} -style=file -i {} \;"
OUTPUT=$(eval "$CMD" 2>&1 | grep error)
if [ -z "$OUTPUT" ]; then
  echo "clang-format check passed! No errors"
  exit 0;
else
  eval "$CMD"
  exit 1;
fi
