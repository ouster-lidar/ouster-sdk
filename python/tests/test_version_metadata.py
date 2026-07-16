"""Tests for package version and native build metadata."""

import os
from typing import Any, Dict

import pytest


@pytest.fixture(scope="module")
def bindings_client():
    return pytest.importorskip("ouster.sdk._bindings.client")


def test_sdk_package_version() -> None:
    """``ouster.sdk.__version__`` contains ``__version__`` from repo-root ``VERSION``."""
    import ouster.sdk as sdk

    v = sdk.__version__
    assert isinstance(v, str)
    assert v

    repo_version_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "VERSION")
    )
    assert os.path.isfile(repo_version_path), f"missing {repo_version_path}"
    ns: Dict[str, Any] = {}
    with open(repo_version_path, encoding="utf-8") as f:
        exec(f.read(), ns)
    expected = ns.get("__version__")
    assert isinstance(expected, str) and expected
    assert expected in v, (
        f"package version {v!r} should contain repo VERSION __version__ {expected!r}"
    )


def test_bindings_build_metadata_are_nonempty_strings_when_present(
    bindings_client,
) -> None:
    build_meta_attrs = (
        "__sdk_version_full__",
        "__build_branch__",
        "__build_hash__",
        "__build_type__",
        "__build_system__",
    )
    for attr in build_meta_attrs:
        if hasattr(bindings_client, attr):
            val = getattr(bindings_client, attr)
            assert isinstance(val, str)
            assert val
