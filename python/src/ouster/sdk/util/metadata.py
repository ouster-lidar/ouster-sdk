"""Miscellaneous utilites."""

import os.path
from pathlib import Path
from typing import Optional, List
from ouster.sdk.core import SensorInfo


data_must_be_a_file_err = "The source parameter must be a path to a file."
meta_must_be_a_file_err = "The metadata parameter must be a path to a file."


def _check_sensor_metadata_for_duplicates(data_path, metas):
    """Check sensor metadata if multiple metadata files are found for the same data path in dir

    Args:
        data_path: data filepath found in the directory
        metas: metadata filepaths found in the directory

    Returns:
        serial number if the metadata files are duplicates else None
    """
    serial_numbers = set()
    for meta in metas:
        # Open each file, read its content, and create a SensorInfo object
        with open(meta) as file:
            meta_content = file.read()
            si = SensorInfo(meta_content)
        if si.sn in serial_numbers:
            s = ["The following metadata files identified for "
                 f"{data_path} contain configuration for the same sensor {si.sn}. Files: "
                 f"{', '.join(sorted((set(metas))))} ",
                 "To resolve this, remove the extra metadata file(s) or specify the metadata "
                 "files manually using the --meta option."]
            raise RuntimeError("\n".join(s))
        serial_numbers.add(si.sn)


def _resolve_metadata_multi_with_prefix_guess(data_path: str) -> List[str]:
    """Look for best-matching metadata files from all json files in the same dir

    Args:
        data_path: filename location with the data, usually .pcap or .bag

    Returns:
        list of metadata json paths guessed with the most common prefix match
    """
    if not os.path.isfile(data_path):
        raise ValueError(data_must_be_a_file_err)

    dirname, pcap_ = os.path.split(data_path)
    if not dirname:
        dirname = os.getcwd()
    # find all .json files in same dir
    options = list(filter(
        lambda f: (Path(dirname) / f).is_file() and
        f.lower().endswith(".json"), os.listdir(dirname)
    ))
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
        # TWS 20240329: previously, this method would return
        # any old JSON file even if there was no common prefix.
        # In my experience, it's almost always an incorrect guess.
        # Now it requires at least a single character to be common.
        return []
    else:
        metas = [
            os.path.join(dirname, b_path) for b_path, _ in filter(
                lambda i: i[1] == best_score, sorted_options)
        ]
        if len(metas) > 1:
            _check_sensor_metadata_for_duplicates(data_path, metas)
        return metas


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
    if meta_path is not None:
        if os.path.isfile(meta_path):
            return meta_path
        raise ValueError(meta_must_be_a_file_err)

    meta_paths = _resolve_metadata_multi_with_prefix_guess(data_path)
    return meta_paths[0] if meta_paths else None


def resolve_metadata_multi(data_path: str) -> List[str]:
    """Look for a metadata files based on the pcap path with multi sensors.

    Args:
        data_path: filename location with the data, usually .pcap or .bag

    Returns:
        list of metadata json paths guessed with the most common prefix match
    """
    return _resolve_metadata_multi_with_prefix_guess(data_path)
