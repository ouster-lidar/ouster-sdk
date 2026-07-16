"""
Copyright (c) 2026, Ouster, Inc.
All rights reserved.

Shared mapping and perception algorithms for Ouster lidar data.
"""
# flake8: noqa (unused imports)

import importlib

from ouster.sdk._kwarg_aliases import install_algorithm_kwarg_aliases

install_algorithm_kwarg_aliases(importlib.import_module("ouster.sdk._bindings.algorithm"))

from ouster.sdk._bindings.algorithm import align_clouds
from ouster.sdk._bindings.algorithm import point_to_plane_align
from ouster.sdk._bindings.algorithm import point_to_point_align
from ouster.sdk._bindings.algorithm import normals
from ouster.sdk._bindings.algorithm import GroundSegConfig
from ouster.sdk._bindings.algorithm import GroundSegEngine
