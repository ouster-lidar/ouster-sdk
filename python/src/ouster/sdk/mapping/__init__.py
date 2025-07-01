"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.

Slam Module
"""
import json
import importlib.resources
from packaging.markers import Marker

from ouster.sdk._bindings.mapping import PoseOptimizer     # noqa: F401
from ouster.sdk._bindings.mapping import SolverConfig      # noqa: F401
from ouster.sdk._bindings.mapping import SamplingMode      # type: ignore # noqa: F401
from ouster.sdk._bindings.mapping import LossFunction      # type: ignore # noqa: F401
from ouster.sdk._bindings.mapping import save_trajectory   # type: ignore # noqa: F401


def get_unsupported():
    result = []
    unsupported = []
    supported = []
    with importlib.resources.open_binary("ouster.sdk.mapping", 'requirements.json') as f:
        requirements = json.loads(f.read())

    for item in requirements:
        marker = Marker(item['marker'])
        if not marker.evaluate():
            unsupported.append(item['name'])
        else:
            supported.append(item['name'])
    for item in unsupported:
        if item not in supported:
            result.append(item)
    return result


mapping_exception = None
try:
    from ouster.sdk.mapping.slam_backend import SlamBackend  # noqa: F401
    from ouster.sdk.mapping.kiss_slam import KissSlam  # noqa: F401
except Exception as e:  # noqa: E722
    mapping_exception = e

if mapping_exception is not None:
    mapping_error = "Mapping not supported on this platform."
    unsupported = get_unsupported()
    if len(unsupported) > 0:
        mapping_error += "The following dependencies are not supported: "
        mapping_error += ", ".join(unsupported)
    mapping_error += f" Additional Info: {mapping_exception}"

    class SlamBackend:  # type: ignore  # noqa
        def __init__(*kwargs):  # noqa
            raise Exception(mapping_error)  # noqa
    class KissSlam:  # type: ignore  # noqa
        def __init__(*kwargs):  # noqa
            raise Exception(mapping_error)  # noqa
else:
    from ouster.sdk.mapping.slam_backend import SlamConfig                    # noqa: F401
    from ouster.sdk.mapping.kiss_slam import KissSlam                         # noqa: F401
    from ouster.sdk.mapping.slam_engine import SlamEngine                     # noqa: F401
    from ouster.sdk.mapping.localization_backend import LocalizationConfig    # noqa: F401
    from ouster.sdk.mapping.kiss_localization import KissLocalization         # noqa: F401
    from ouster.sdk.mapping.localization_engine import LocalizationEngine     # noqa: F401
