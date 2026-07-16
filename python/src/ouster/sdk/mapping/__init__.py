"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.

This module provides tools and classes for SLAM, localization, and mapping utilities for Ouster lidar data in Python.
It helps building maps, track poses, and process mapping results.
"""
# flake8: noqa (unused imports)

import importlib

from ouster.sdk._kwarg_aliases import install_mapping_kwarg_aliases

install_mapping_kwarg_aliases(importlib.import_module("ouster.sdk._bindings.mapping"))

from ouster.sdk._bindings.mapping import SlamConfig
from ouster.sdk._bindings.mapping import LIOSlamConfig
from ouster.sdk._bindings.mapping import SlamEngine
from ouster.sdk._bindings.mapping import LIOLocalizationConfig
from ouster.sdk._bindings.mapping import LocalizationConfig
from ouster.sdk._bindings.mapping import LocalizationEngine
from ouster.sdk._bindings.mapping import DeskewMethod
from ouster.sdk._bindings.mapping import ConstantVelocityDeskewMethod
from ouster.sdk._bindings.mapping import DeskewMethodFactory
from ouster.sdk._bindings.mapping import ActiveTimeCorrection

from ouster.sdk._bindings.mapping import PoseOptimizer
from ouster.sdk._bindings.mapping import SolverConfig
from ouster.sdk._bindings.mapping import SamplingMode
from ouster.sdk._bindings.mapping import LossFunction
from ouster.sdk._bindings.mapping import save_trajectory

# Constraint classes
from ouster.sdk._bindings.mapping import AbsolutePoseConstraint    # type: ignore # noqa: F401
from ouster.sdk._bindings.mapping import AbsolutePointConstraint   # type: ignore # noqa: F401
from ouster.sdk._bindings.mapping import PoseToPoseConstraint      # type: ignore # noqa: F401
from ouster.sdk._bindings.mapping import PointToPointConstraint    # type: ignore # noqa: F401
from ouster.sdk._bindings.mapping import ICPRegistration              # type: ignore # noqa: F401
from ouster.sdk._bindings.mapping import AdaptiveThreshold         # type: ignore # noqa: F401
