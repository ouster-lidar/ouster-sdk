import sys
from .borg import Borg
from typing import List


class CliArgs(Borg):
    """CliArgs is a Borg-pattern class designed to provide *all* CLI args,
    regardless of whether a CliRunner was used (e.g. during testing) or not.
    This is necessary, unfortunately, because it's not always possible to tell
    whether a command was invoked with a --help option.
    """

    def __init__(self, args=None):
        super().__init__()
        if args is not None:
            self.args = args
        else:
            if not hasattr(self, 'args'):
                self.args = sys.argv[1:]

    def has_any_of(self, flags: List[str]) -> bool:
        return bool(set(self.args) & set(flags))

    def __str__(self):
        return f'CliArgs({self.args})'
