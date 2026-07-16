# Get the directory of the current script
$DIR = Split-Path -Parent $MyInvocation.MyCommand.Path

# Normalize Completion Environment Variables
# Check if any completion var is set and map it to _DEV_PY_COMPLETE
$CompletionValue = $env:_DEV_COMPLETE
if (-not $CompletionValue) { $CompletionValue = $env:_DEV_PS1_COMPLETE }

if ($CompletionValue) {
    $env:_DEV_PY_COMPLETE = $CompletionValue
    # Ensure Click knows the program name being invoked
    if (-not $env:DEV_PROG_NAME) { $env:DEV_PROG_NAME = "dev.ps1" }
}

# Skip expensive context initialization during tab-completion
if (-not $env:_DEV_PY_COMPLETE) {
    if (-not $env:DEV_PERSISTENT_DIR) {
        $env:DEV_PERSISTENT_DIR = python3 -c "import sys; sys.path.append('$DIR/dev_script_library'); import context; click_context = context.ClickContext(None); print(click_context.dev_persistent_dir)"
    }

    if (-not (Test-Path $env:DEV_PERSISTENT_DIR)) {
        New-Item -ItemType Directory -Force -Path $env:DEV_PERSISTENT_DIR | Out-Null
    }
}

# Execute the actual python script
python3 "$DIR/dev.py" @args
