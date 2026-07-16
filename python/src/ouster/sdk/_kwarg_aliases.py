"""
Install deprecated keyword-argument aliases for renamed scan/frame parameters.

This module is the wiring layer for the scan -> frame terminology migration. It
takes the generic deprecation primitives from :mod:`ouster.sdk._deprecation`
and applies them to every concrete function, method and constructor across the
SDK whose keyword arguments were renamed, so that legacy keyword names (e.g.
``scan=``) keep working with a :class:`FutureWarning`.

Each ``install_*_kwarg_aliases`` function patches one SDK subpackage and is
called once from that subpackage's ``__init__`` at import time. The functions
operating on compiled bindings (``client``, ``algorithm``, ``mapping``,
``osf``, ``perception``) take the relevant ``_bindings`` module as an argument
and patch it in place; the pure-Python ones (``core``, ``viz``) import their
targets directly.

All binding patches are skipped during nanobind stub generation — see
:func:`_skip_binding_patches` for why.
"""

from __future__ import annotations

import os
from types import ModuleType
from typing import Iterable

from ouster.sdk._deprecation import (
    SCAN_FRAME_KWARG_ALIASES,
    patch_init_deprecated_kwargs,
    patch_method_deprecated_kwargs,
    wrap_deprecated_kwargs,
    wrap_nb_positional_deprecated_kwargs,
)


def _skip_binding_patches() -> bool:
    """Whether to skip patching the nanobind binding classes/functions.

    During nanobind stub generation the binding objects are introspected to
    produce ``.pyi`` files. The deprecated-kwarg wrappers replace typed nanobind
    methods/functions with untyped ``(*args, **kwargs)`` shims, which would
    corrupt the generated stubs (and break type checking of all call sites).
    The stubgen build step sets ``OUSTER_SDK_GENERATING_STUBS=1`` so the
    pristine, typed signatures are captured; the wrappers are still applied
    normally at runtime.
    """
    return os.environ.get("OUSTER_SDK_GENERATING_STUBS") == "1"


def _patch_functions(module: ModuleType, names: Iterable[str], **aliases: str) -> None:
    """Wrap each named free function on ``module`` in place with kwarg aliases.

    Args:
        module: The module whose attributes are replaced with wrapped versions.
        names: Names of free functions on ``module`` to patch.
        **aliases: Deprecated-name -> new-name pairs to accept, forwarded to
            :func:`~ouster.sdk._deprecation.wrap_deprecated_kwargs`.
    """
    for name in names:
        setattr(module, name, wrap_deprecated_kwargs(getattr(module, name), **aliases))


def install_client_kwarg_aliases(client: ModuleType) -> None:
    """Install scan -> frame kwarg aliases on the ``client`` binding module.

    Wraps the frame-operation free functions, the positional-only
    ``frame_to_packets`` binding, and the ``__call__`` of the XYZ lookup-table
    classes so they accept their deprecated keyword names. Does nothing during
    stub generation.

    Args:
        client: The ``ouster.sdk._bindings.client`` module, patched in place.
    """
    if _skip_binding_patches():
        return
    aliases = SCAN_FRAME_KWARG_ALIASES
    _patch_functions(
        client,
        [
            "clip",
            "filter_field",
            "_frame_ops_filter_uv",
            "_frame_ops_mask",
            "select_by_index",
            "reduce_by_factor",
        ],
        **aliases,
    )
    client.frame_to_packets = wrap_nb_positional_deprecated_kwargs(  # type: ignore[attr-defined]
        client.frame_to_packets,
        ["lidar_frame", "packet_format", "init_id", "prod_sn"],
        lidar_scan="lidar_frame",
    )
    # NOTE: FrameSet's deprecated ``scans`` kwarg is handled by a dedicated
    # constructor overload in the C++ binding (see lidar_frame.cpp). nanobind
    # classes construct in their ``tp_new`` slot, so reassigning ``__init__``
    # via patch_init_deprecated_kwargs would be a silent no-op here.
    patch_method_deprecated_kwargs(client.XYZLutFloat, "__call__", **aliases)
    patch_method_deprecated_kwargs(client.XYZLut, "__call__", **aliases)


def install_algorithm_kwarg_aliases(algorithm: ModuleType) -> None:
    """Install scan -> frame kwarg aliases on the ``algorithm`` binding module.

    Patches the ``align_clouds`` free function and ``GroundSegEngine.update``.
    Does nothing during stub generation.

    Args:
        algorithm: The ``ouster.sdk._bindings.algorithm`` module, patched in place.
    """
    if _skip_binding_patches():
        return
    aliases = SCAN_FRAME_KWARG_ALIASES
    _patch_functions(algorithm, ["align_clouds"], **aliases)
    patch_method_deprecated_kwargs(algorithm.GroundSegEngine, "update", **aliases)


def install_mapping_kwarg_aliases(mapping: ModuleType) -> None:
    """Install scan -> frame kwarg aliases on the ``mapping`` binding module.

    Patches the ``update`` methods of ``SlamEngine``, ``LocalizationEngine`` and
    ``ActiveTimeCorrection`` plus ``ActiveTimeCorrection.reset``. Does nothing
    during stub generation.

    Args:
        mapping: The ``ouster.sdk._bindings.mapping`` module, patched in place.
    """
    if _skip_binding_patches():
        return
    aliases = SCAN_FRAME_KWARG_ALIASES
    patch_method_deprecated_kwargs(mapping.SlamEngine, "update", **aliases)
    patch_method_deprecated_kwargs(mapping.LocalizationEngine, "update", **aliases)
    patch_method_deprecated_kwargs(mapping.ActiveTimeCorrection, "update", **aliases)
    patch_method_deprecated_kwargs(mapping.ActiveTimeCorrection, "reset", **aliases)


def install_osf_kwarg_aliases(osf: ModuleType) -> None:
    """Install scan -> frame kwarg aliases on the ``osf`` binding module.

    Patches the ``slice_and_cast`` free function and the ``save`` methods of
    ``Writer`` and ``AsyncWriter``. Does nothing during stub generation.

    The ``Encoder`` constructor's deprecated ``lidar_scan_encoder`` keyword is
    intentionally *not* handled here; because ``Encoder`` is a nanobind class it
    is deprecated via a C++ constructor overload instead (see the note below).

    Args:
        osf: The ``ouster.sdk._bindings.osf`` module, patched in place.
    """
    if _skip_binding_patches():
        return
    aliases = SCAN_FRAME_KWARG_ALIASES
    _patch_functions(osf, ["slice_and_cast"], **aliases)
    for cls in (osf.Writer, osf.AsyncWriter):
        patch_method_deprecated_kwargs(cls, "save", **aliases)
    # NOTE: Encoder's deprecated ``lidar_scan_encoder`` kwarg is handled by a
    # dedicated constructor overload in the C++ binding (see _osf.cpp). Encoder
    # is a nanobind class that constructs in its ``tp_new`` slot, so reassigning
    # ``__init__`` via patch_init_deprecated_kwargs would be a silent no-op here.


def install_perception_kwarg_aliases(perception: ModuleType) -> None:
    """Install scan -> frame kwarg aliases on the ``perception`` binding module.

    Patches ``DetectionEngine.update``. Does nothing during stub generation.

    Args:
        perception: The ``ouster.sdk._bindings.perception`` module, patched in
            place.
    """
    if _skip_binding_patches():
        return
    aliases = SCAN_FRAME_KWARG_ALIASES
    patch_method_deprecated_kwargs(perception.DetectionEngine, "update", **aliases)


def install_core_python_kwarg_aliases() -> None:
    """Install scan -> frame kwarg aliases on the pure-Python ``core`` modules.

    Unlike the binding installers above, this takes no module argument: it
    imports its targets from :mod:`ouster.sdk.core` directly and patches the
    pure-Python free functions, frame-ops helpers and ``*FrameSetSource``
    constructors in place. It also re-exports the patched
    ``first_valid_column_pose`` / ``last_valid_column_pose`` helpers through the
    package namespace so callers importing them from ``ouster.sdk.core`` get the
    wrapped versions.

    These objects are not nanobind bindings, so this installer is *not* gated on
    :func:`_skip_binding_patches`; it runs even during stub generation.
    """
    from ouster.sdk.core import core as core_mod
    from ouster.sdk.core import frame_ops
    from ouster.sdk.core import clipped_frame_set_source
    from ouster.sdk.core import masked_frame_set_source
    from ouster.sdk.core import reduced_frame_set_source
    from ouster.sdk.core import _digest
    from ouster.sdk.util import parsing

    core_mod.first_valid_column_pose = wrap_deprecated_kwargs(
        core_mod.first_valid_column_pose, scan="frame")
    core_mod.last_valid_column_pose = wrap_deprecated_kwargs(
        core_mod.last_valid_column_pose, scan="frame")
    # Re-export patched helpers through the package namespace.
    import ouster.sdk.core as core_pkg
    core_pkg.first_valid_column_pose = core_mod.first_valid_column_pose
    core_pkg.last_valid_column_pose = core_mod.last_valid_column_pose

    for name in ("filter_uv", "filter_xyz", "mask", "select_by_index", "reduce_by_factor"):
        setattr(frame_ops, name, wrap_deprecated_kwargs(getattr(frame_ops, name), scan="frame"))

    patch_init_deprecated_kwargs(
        clipped_frame_set_source.ClippedFrameSetSource,
        scan_source="frame_set_source",
    )
    patch_init_deprecated_kwargs(
        masked_frame_set_source.MaskedFrameSetSource,
        scan_source="frame_set_source",
    )
    patch_init_deprecated_kwargs(
        reduced_frame_set_source.ReducedFrameSetSource,
        scan_source="frame_set_source",
    )

    patch_init_deprecated_kwargs(_digest.StreamDigest, scans="frames")
    parsing.cut_raw32_words = wrap_deprecated_kwargs(parsing.cut_raw32_words, ls="lf")


def install_viz_python_kwarg_aliases() -> None:
    """Install scan -> frame kwarg aliases on the pure-Python ``viz`` modules.

    Patches the ``update`` methods (and a couple of constructors) of the
    visualizer, its model, the accumulators and the track types. Because these
    signatures differ from one another, only the relevant subset of aliases is
    applied to each (for example ``scans`` -> ``frames`` and
    ``scan_num`` -> ``frame_num`` on accumulator ``update`` methods).

    Like :func:`install_core_python_kwarg_aliases`, this patches pure-Python
    objects, takes no arguments, and is not gated on
    :func:`_skip_binding_patches`.
    """
    from ouster.sdk.viz import core as viz_core
    from ouster.sdk.viz import model as viz_model
    from ouster.sdk.viz import accum_base, accumulators, track, map_accumulator, frames_accumulator

    patch_method_deprecated_kwargs(
        viz_core.LidarFrameViz,
        "update",
        scans="frames",
        scan_num="frame_num",
        last_n_scans="last_n_frame_sets",
    )
    patch_init_deprecated_kwargs(
        viz_core.SimpleViz,
        _override_lidarscanviz="_override_lidar_frame_viz",
    )
    patch_method_deprecated_kwargs(
        viz_model.LidarFrameVizModel,
        "update",
        scans="frames",
        new_scans="new_frames",
    )
    for cls in (
        accum_base.AccumulatorBase,
        map_accumulator.MapAccumulator,
        frames_accumulator.FramesAccumulator,
    ):
        patch_method_deprecated_kwargs(
            cls,
            "update",
            scans="frames",
            scan_num="frame_num",
        )
    patch_method_deprecated_kwargs(
        accumulators.LidarFrameVizAccumulators,
        "update",
        scans="frames",
        scan_num="frame_num",
    )
    patch_method_deprecated_kwargs(
        track.Track,
        "update",
        scan="frame",
    )
    patch_method_deprecated_kwargs(
        track.MultiTrack,
        "update",
        scans="frames",
        scan_num="frame_num",
    )
    patch_init_deprecated_kwargs(track.FrameRecord, scan="frame")
