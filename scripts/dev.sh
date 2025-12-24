#! /bin/bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$DEV_PERSISTANT_DIR"];
then
    DEV_PERSISTANT_DIR=$(python3 -c "import sys; \
                             sys.path.append('$DIR/dev_script_library'); \
                             import context; \
                             click_context = context.ClickContext(None); \
                             print(click_context.dev_persistant_dir)")
fi
if ! [ -d "$DEV_PERSISTANT_DIR" ]; then
    mkdir -p $DEV_PERSISTANT_DIR
fi

python3 $DIR/dev.py $@
