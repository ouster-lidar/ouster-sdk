"""SDK build metadata from native bindings for ouster-cli --version."""
from __future__ import annotations

from typing import Any


def _metadata_keys() -> tuple[tuple[str, str], ...]:
    # Order for CLI display (matches ouster/impl/build.h + generated build.cpp).
    return (
        ("__sdk_version_full__", "SDK version (native, CMake + git)"),
        ("__build_branch__", "Git branch"),
        ("__build_hash__", "Git SHA"),
        ("__build_type__", "CMake build type"),
        ("__build_system__", "Host OS (CMake)"),
    )


def _sdk_build_metadata() -> dict[str, Any]:
    """
    Fields exported on ``ouster.sdk._bindings.client`` from ``ouster/impl/build.h``.
    Empty if the extension is missing or not built with ``ouster_build``.
    """
    try:
        import ouster.sdk._bindings.client as _client  # type: ignore[import-not-found]
    except ImportError:
        return {}
    out: dict[str, Any] = {}
    for attr, _ in _metadata_keys():
        if hasattr(_client, attr):
            val = getattr(_client, attr)
            if isinstance(val, str) and val:
                out[attr] = val
    return out


def sdk_build_metadata_display_order() -> tuple[tuple[str, str], ...]:
    """(attribute name, label) pairs for printing."""
    return _metadata_keys()
