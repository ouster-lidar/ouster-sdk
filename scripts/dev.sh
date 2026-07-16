#! /bin/bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# When the shell requests tab-completion it sets a _DEV_*_COMPLETE env var.
# The zsh/bash completion scripts set _DEV_COMPLETE; normalise that to
# _DEV_SH_COMPLETE so it matches the prog_name "dev.sh" that Click sees
# when cli.main(prog_name="dev.sh") is called.
if [ -n "${_DEV_COMPLETE:-}" ] && [ -z "${_DEV_SH_COMPLETE:-}" ]; then
    export _DEV_SH_COMPLETE="${_DEV_COMPLETE}"
fi
if [ -n "${_DEV_SH_COMPLETE:-}" ]; then
    export DEV_PROG_NAME="${DEV_PROG_NAME:-dev.sh}"
fi

# Skip expensive context initialisation during tab-completion.
if [ -z "${_DEV_PY_COMPLETE}" ]; then
    if [ -z "$DEV_PERSISTENT_DIR" ]; then
        DEV_PERSISTENT_DIR=$(python3 -c "import sys; \
                                 sys.path.append('${DIR}/dev_script_library'); \
                                 import context; \
                                 click_context = context.ClickContext(None); \
                                 print(click_context.dev_persistent_dir)")
    fi
    if ! [ -d "$DEV_PERSISTENT_DIR" ]; then
        mkdir -p "$DEV_PERSISTENT_DIR"
    fi
fi

python3 "$DIR/dev.py" "$@"
