from typing import Dict, Any


class Borg:
    """https://github.com/faif/python-patterns/blob/master/patterns/creational/borg.py"""
    _shared_state: Dict[str, Any] = {}

    def __init__(self):
        self.__dict__ = self._shared_state
