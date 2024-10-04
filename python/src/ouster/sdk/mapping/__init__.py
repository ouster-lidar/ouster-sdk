"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.

Slam Module
"""
import json
import importlib.resources
from packaging.markers import Marker


def get_unsupported():
    result = []
    with importlib.resources.open_binary("ouster.sdk.mapping", 'requirements.json') as f:
        requirements = json.loads(f.read())

    for item in requirements:
        marker = Marker(requirements[item]['marker'])
        if not marker.evaluate():
            result.append(item)
    return result


mapping_exception = None
try:
    from .slam_backend import SLAMBackend  # noqa: F401
    from .kiss_backend import KissBackend  # noqa: F401
except Exception as e:  # noqa: E722
    mapping_exception = e

if mapping_exception is not None:
    mapping_error = "Mapping not supported on this platform."
    unsupported = get_unsupported()
    if len(unsupported) > 0:
        mapping_error += "The following dependencies are not supported: "
        mapping_error += ", ".join(unsupported)
    mapping_error += f" Additional Info: {mapping_exception}"
    class SLAMBackend:  # type: ignore  # noqa
        def __init__(*kwargs):  # noqa
            raise Exception(mapping_error)  # noqa
    class KissBackend:  # type: ignore  # noqa
        def __init__(*kwargs):  # noqa
            raise Exception(mapping_error)  # noqa
