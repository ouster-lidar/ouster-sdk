"""
Copyright (c) 2026, Ouster, Inc.
All rights reserved.
"""
from typing import Any, Dict, FrozenSet, Tuple, Type
import warnings

from ouster.sdk._bindings.perception import ClassicDetectionConfig, DetectionConfig

_ClassicFields = frozenset({
    "save_instance_id_fields",
    "cluster_filter_min_side_length",
    "cluster_filter_min_vertical_size",
    "cluster_filter_max_volume",
    "cluster_filter_max_side_length",
})

_KIND_REGISTRY: Dict[str, Tuple[Type[DetectionConfig], FrozenSet[str]]] = {
    "classic": (ClassicDetectionConfig, _ClassicFields),
}


def create_detection_config(kind: str = "classic", **kwargs: Any) -> DetectionConfig:
    """Build a :class:`DetectionConfig` for a given engine ``kind``.

    Parameters are applied by name to the config object. Base
    :class:`DetectionConfig` fields (e.g. ``save_instance_id_fields``) are
    accepted for kinds that use a derived config (such as ``"classic"``).

    Args:
        kind: Engine identifier, matching :meth:`DetectionEngine.create` (e.g.
            ``"classic"``).
        **kwargs: Config attributes to set on the new instance.

    Returns:
        A config instance appropriate for ``kind``.

    Raises:
        ValueError: If ``kind`` is not registered.
    """
    key = kind.lower()
    try:
        cls, fields = _KIND_REGISTRY[key]
    except KeyError:
        known = ", ".join(sorted(_KIND_REGISTRY))
        raise ValueError(
            f"Unknown detection engine kind {kind!r}; expected one of: {known}"
        ) from None

    config = cls()
    for name, value in kwargs.items():
        if name in fields:
            setattr(config, name, value)
        else:
            warnings.warn(
                f"create_detection_config(kind={kind!r}): no config field "
                f"{name!r}; value ignored",
                stacklevel=2,
            )
    return config


DetectionConfig.create = staticmethod(create_detection_config)  # type: ignore
