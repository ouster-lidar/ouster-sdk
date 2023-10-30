"""Miscellaneous utilites."""

import os
from typing import Optional, List, Any
from packaging import version
import requests
import re


def resolve_metadata_multi_with_prefix_guess(data_path: str) -> List[str]:
    """Look for best-matching metadata files from all json files in the same dir

    Args:
        data_path: filename location with the data, usually .pcap or .bag

    Returns:
        list of metadata json paths guessed with the most common prefix match
    """
    dirname, pcap_ = os.path.split(data_path)
    if not dirname:
        dirname = os.getcwd()
    # find all .json in same dir
    options = list(filter(lambda f: f.endswith(".json"), os.listdir(dirname)))
    # for each json name, find how many characters are in common
    option_scores = map(lambda f: len(os.path.commonprefix([f, pcap_])),
                        options)
    if not options:
        return []
    # select all jsons with the longest common prefix of equal size
    sorted_options = sorted(zip(options, option_scores),
                           key=lambda i: i[1],
                           reverse=True)
    best_score = sorted_options[0][1]
    if not best_score:
        # return a single json if there is no files with commonprefix
        # because it's probably not a multi-sensor recording
        return [os.path.join(dirname, sorted_options[0][0])]
    else:
        return [
            os.path.join(dirname, b_path) for b_path, _ in filter(
                lambda i: i[1] == best_score, sorted_options)
        ]


def resolve_metadata(data_path: str,
                     meta_path: Optional[str] = None) -> Optional[str]:
    """Look for a metadata file based on the data path if needed.

    Convenient to use in CLI tools when --meta param can be omitted
    in lots of trivial cases when pcap filename has the same prefix
    as the metadata json filename.

    Args:
        data_path: filename location with the data, usually .pcap or .bag
                   that is used to search metadata with the most common
                   prefix file
        meta_path: the pass through metadata path, if set guessing and
                   search for other metadata jsons is skipped

    Returns:
        metadata json paths guessed with the most common prefix match or passed
        through from `meta_path` parameter
    """
    if meta_path is None:
        meta_paths = resolve_metadata_multi_with_prefix_guess(data_path)
        meta_path = meta_paths[0] if meta_paths else ""
        if os.path.exists(meta_path):
            return meta_path
    elif os.path.exists(meta_path):
        return meta_path
    return None


def resolve_metadata_multi(data_path: str) -> List[str]:
    """Look for a metadata files based on the pcap path with multi sensors.

    Args:
        data_path: filename location with the data, usually .pcap or .bag

    Returns:
        list of metadata json paths guessed with the most common prefix match
    """
    return resolve_metadata_multi_with_prefix_guess(data_path)


def firmware_version(hostname: str) -> Any:
    firmware_version_endpoint = f"http://{hostname}/api/v1/system/firmware"
    response = requests.get(firmware_version_endpoint)
    if response.status_code != requests.codes.ok:
        raise RuntimeError("Could not get sensor firmware version")

    match = re.search("v(\\d+).(\\d+)\\.(\\d+)", response.text)
    if match:
        return version.Version(".".join(
            [match.group(1), match.group(2),
             match.group(3)]))
    else:
        raise RuntimeError(
            f"Could not get sensor firmware version from {response.text}")
    return None
