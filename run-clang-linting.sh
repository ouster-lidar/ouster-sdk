#!/bin/bash
# Ignore the ./ouster_client/include/optional-lite/nonstd/optional.hpp file while formatting files
CMD="find . -regex '.*\.\(cpp\|hpp\|cu\|c\|h\)' -not -path ./ouster_client/include/optional-lite/nonstd/optional.hpp \
-exec clang-format --Werror --dry-run -style=file -i {} \;"
OUTPUT=$(eval "$CMD" 2>&1 | grep error)
if [ -z "$OUTPUT" ]; then
  echo "clang-format check passed! No errors"
  exit 0;
else
  eval "$CMD"
  exit 1;
fi