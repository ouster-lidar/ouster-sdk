"""
Copyright (c) 2025, Ouster, Inc.
All rights reserved.

Deprecation utilities for Python bindings.
This module provides deprecation warnings for C++ symbols that have been deprecated.
"""

import warnings
from typing import Dict, Generic, TypeVar


T = TypeVar('T')  # Generic type for return values


def _get_deprecation_message(old_name: str, new_name: str, last_supported_version: str) -> str:
    """Generate a standard deprecation warning message.

    Args:
        old_name: The deprecated name
        new_name: The new name to use instead
        last_supported_version: The version after which this alias will be removed

    Returns:
        A formatted deprecation warning message
    """
    return (f"{old_name} is deprecated: Use {new_name} instead. "
            f"The last supported version for this will be {last_supported_version}.")


def deprecated_alias(old_name: str, new_name: str, obj: object,
                  module_dict: Dict[str, object],
                  last_supported_version: str) -> None:
    """
    Create a deprecated alias for an object with appropriate warning.

    Args:
        old_name: The deprecated name
        new_name: The new name to use instead
        obj: The object being aliased
        module_dict: The module's __dict__ to add the alias to
        last_supported_version: The version after which this alias will be removed.
    """
    version = last_supported_version

    def _deprecated_wrapper(*args, **kwargs):
        warnings.warn(
            _get_deprecation_message(old_name, new_name, version),
            DeprecationWarning,
            stacklevel=2
        )
        if callable(obj):
            return obj(*args, **kwargs)
        return obj

    # For non-callable objects, we need a different approach
    if not callable(obj):
        class DeprecatedAttribute(Generic[T]):
            def __init__(self, value: T, old_name: str, new_name: str, version: str):
                self._value = value
                self._old_name = old_name
                self._new_name = new_name
                self._version = version
                self._warned = False

            def _warn_if_needed(self):
                if not self._warned:
                    warnings.warn(
                        _get_deprecation_message(self._old_name, self._new_name, self._version),
                        DeprecationWarning,
                        stacklevel=3
                    )
                    self._warned = True

            def __getattr__(self, name):
                self._warn_if_needed()
                return getattr(self._value, name)

            def __repr__(self):
                self._warn_if_needed()
                return repr(self._value)

            def __str__(self):
                self._warn_if_needed()
                return str(self._value)

            def __eq__(self, other):
                self._warn_if_needed()
                return self._value == other

            def __hash__(self):
                self._warn_if_needed()
                return hash(self._value)

            def __ne__(self, other):
                self._warn_if_needed()
                return self._value != other

            def __bool__(self):
                self._warn_if_needed()
                return bool(self._value)

            def __int__(self):
                self._warn_if_needed()
                return int(self._value)

            def __float__(self):
                self._warn_if_needed()
                return float(self._value)

        module_dict[old_name] = DeprecatedAttribute(obj, old_name, new_name, version)
    else:
        module_dict[old_name] = _deprecated_wrapper
