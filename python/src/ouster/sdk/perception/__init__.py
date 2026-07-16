"""
Copyright (c) 2025, Ouster, Inc.
All rights reserved.
"""
# flake8: noqa (unused imports)

import importlib

from ouster.sdk._kwarg_aliases import install_perception_kwarg_aliases

install_perception_kwarg_aliases(importlib.import_module("ouster.sdk._bindings.perception"))

from ouster.sdk._bindings.perception import ClassicDetectionConfig
from ouster.sdk._bindings.perception import DetectionConfig
from ouster.sdk._bindings.perception import DetectionEngine
from ouster.sdk._bindings.algorithm import normals as _normals
from ouster.sdk._deprecation import warn_deprecated
from .create_config import create_detection_config


def normals(*args, **kwargs):
    warn_deprecated(
        "ouster.sdk.perception.normals is deprecated: "
        "Use ouster.sdk.algorithm.normals instead. "
        "The last supported version for this will be 1.0."
    )
    return _normals(*args, **kwargs)
