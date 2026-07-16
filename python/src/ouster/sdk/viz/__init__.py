"""
Copyright (c) 2023, Ouster, Inc.
All rights reserved.

This module provides OpenGL-based visualization for Ouster lidar data.
It includes the PointViz renderer plus higher-level helpers such as
SimpleViz and LidarFrameViz for displaying LidarFrames and range images.
"""
# flake8: noqa (unused imports)

from ouster.sdk._bindings.viz import MouseButton
from ouster.sdk._bindings.viz import MouseButtonEvent
from ouster.sdk._bindings.viz import EventModifierKeys
from ouster.sdk._bindings.viz import PointVizNotRunningError
from ouster.sdk._bindings.viz import PointViz
from ouster.sdk._bindings.viz import Cloud
from ouster.sdk._bindings.viz import Image
from ouster.sdk._bindings.viz import Vertex3f
from ouster.sdk._bindings.viz import Mesh
from ouster.sdk._bindings.viz import Cuboid
from ouster.sdk._bindings.viz import Label
from ouster.sdk._bindings.viz import Lines
from ouster.sdk._bindings.viz import ObjectOverlay
from ouster.sdk._bindings.viz import WindowCtx
from ouster.sdk._bindings.viz import Camera
from ouster.sdk._bindings.viz import TargetDisplay
from ouster.sdk._bindings.viz import add_default_controls
from ouster.sdk._bindings.viz import calref_palette
from ouster.sdk._bindings.viz import spezia_palette
from ouster.sdk._bindings.viz import grey_palette
from ouster.sdk._bindings.viz import viridis_palette
from ouster.sdk._bindings.viz import magma_palette

from .view_mode import ImageMode
from .view_mode import CloudMode
from .view_mode import ImageCloudMode

from .core import LidarFrameViz
from .core import SimpleViz
from .core import lf_show
from .core import ImuVisualizationConfig
from .core import CloudPaletteItem
from .core import VizExtraMode

from .accumulators import LidarFrameVizAccumulatorsConfig

from .util import AxisWithLabel

from ouster.sdk._deprecation import deprecated_alias
deprecated_alias("LidarScanViz", "LidarFrameViz", LidarFrameViz, globals(), "1.0")
deprecated_alias("LidarScanVizAccumulatorsConfig", "LidarFrameVizAccumulatorsConfig", LidarFrameVizAccumulatorsConfig, globals(), "1.0")
deprecated_alias("ls_show", "lf_show", lf_show, globals(), "1.0")

from ouster.sdk._kwarg_aliases import install_viz_python_kwarg_aliases
install_viz_python_kwarg_aliases()
