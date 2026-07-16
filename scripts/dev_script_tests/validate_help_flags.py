"""
Validate that every public Click command exposed by the Dev Script library
responds correctly to the --help flag.

Run this script directly to verify the full CLI tree before submitting
changes.  It imports the CLI tree through the same bootstrap path used by
dev.py so that the validation is always consistent with what end-users see.

Exit codes
----------
0  All commands produced a non-empty help string and exited cleanly.
1  One or more commands failed the --help check.
"""

import sys
import os
import importlib
from unittest.mock import patch

from click.testing import CliRunner

# Ensure the library root is importable without requiring an editable install.
_SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

# Bootstrap the dev CLI without triggering any real subprocess calls.
with patch("os.path.exists", return_value=False):
    dev = importlib.import_module("dev")


def _collect_commands(group, prefix=""):
    """Recursively yield (full_name, command) for every leaf command."""
    for name, cmd in group.commands.items():
        full_name = f"{prefix} {name}".strip()
        if hasattr(cmd, "commands"):
            yield from _collect_commands(cmd, prefix=full_name)
        else:
            yield full_name, cmd


def validate_all_help_flags():
    runner = CliRunner()
    failures = []

    commands = list(_collect_commands(dev.cli))
    if not commands:
        print("ERROR: No commands discovered in the CLI tree.", file=sys.stderr)
        sys.exit(1)

    print(f"Validating --help for {len(commands)} public command(s)...\n")

    for full_name, _cmd in commands:
        tokens = full_name.split() + ["--help"]
        result = runner.invoke(dev.cli, tokens)

        ok = result.exit_code == 0 and result.output.strip()
        status = "PASS" if ok else "FAIL"
        print(f"  [{status}]  dev {full_name} --help")

        if not ok:
            failures.append(full_name)
            print(f"         exit_code={result.exit_code}")
            if result.output:
                for line in result.output.splitlines()[:10]:
                    print(f"         {line}")
            if result.exception:
                print(f"         exception: {result.exception}")

    print()
    if failures:
        print(f"FAILED: {len(failures)} command(s) did not pass --help validation:")
        for name in failures:
            print(f"  - {name}")
        sys.exit(1)
    else:
        print(f"All {len(commands)} command(s) passed --help validation.")


if __name__ == "__main__":
    validate_all_help_flags()
