import logging
import sys
import numpy as np
from pathlib import Path

import ouster.sdk.osf as osf
from ouster.sdk import client
from ouster.sdk.open_source import open_source

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('mapping_tool')


def find_pcap_files(directory):
    directory_path = Path(directory)
    return [str(file_path) for file_path in directory_path.glob('*.pcap')]


def update_scan_ts(scan, ts_diff):
    for i, timestamp in enumerate(scan.timestamp):
        if timestamp != 0:
            scan.timestamp[i] -= ts_diff


def write_osf_from_source(data_source, osf_writer, ts_diff, last_ts):
    cal_diff = True

    for scan in data_source:
        if last_ts != 0 and cal_diff:
            cur_ts = client.first_valid_column_ts(scan)
            ts_diff = cur_ts - last_ts
            cal_diff = False
            continue

        update_scan_ts(scan, ts_diff)
        ts = client.first_valid_column_ts(scan)
        osf_writer.save(0, scan, ts)

    return ts


def pcaps_to_osfs(files_path, osf_out_name):
    """
    One pcap to One osf conversion. Use it in stitch.py only.
    Because stitch.py output one OSF file and OSF has to have increasing ts. So
    we have to manipulate timestamp in scan here too

    Require the discreted pcap files are recorded while sensor is always on, that
    ensure the timestamps are in the similiar ranges and ascendant.

    Unknow behavior if sensor is restarted. Timestampes in very different ranges
    may cause int overflow issue or other unknow issues.
    """
    osf_out_name = Path(osf_out_name)
    pcap_files = find_pcap_files(files_path)
    sorted_files = sorted(pcap_files)
    ts_diff = 0
    last_ts = 0
    out_dir = osf_out_name.parent
    file_wo_ext = osf_out_name.stem
    outfile_ext = osf_out_name.suffix

    new_field_types = dict({
        client.ChanField.RANGE: np.dtype('uint32'),
        client.ChanField.SIGNAL: np.dtype('uint16'),
        client.ChanField.REFLECTIVITY: np.dtype('uint16')})

    for idx, file in enumerate(sorted_files):
        data_source = open_source(file, sensor_idx=0)
        osf_filename = str(
                out_dir / Path(file_wo_ext + str(idx + 1) + outfile_ext))
        logger.info(f"Convert {file} to {osf_filename} ")
        osf_writer = osf.Writer(osf_filename, data_source.metadata, new_field_types)

        last_ts = write_osf_from_source(
            data_source, osf_writer, ts_diff, last_ts)

        osf_writer.close()


def pcaps_to_osf(files_path, osf_out_name):
    """
    Combined pcap files into one osf file. Manipulate ts and make discrete pcap
    files into a continous osf file.

    Require the discreted pcap files are recorded while sensor is always on, that
    ensure the timestamps are in the similiar ranges and ascendant.

    Unknow behavior if sensor is restarted. Timestampes in very different ranges
    may cause int overflow issue or other unknow issues.
    """
    pcap_files = find_pcap_files(files_path)
    sorted_files = sorted(pcap_files)
    ts_diff = 0
    last_ts = 0
    new_field_types = dict({
        client.ChanField.RANGE: np.dtype('uint32'),
        client.ChanField.SIGNAL: np.dtype('uint16'),
        client.ChanField.REFLECTIVITY: np.dtype('uint16')})

    peak_reader = open_source(str(pcap_files[0]))
    info = peak_reader.metadata
    del peak_reader
    osf_writer = osf.Writer(osf_out_name, info, new_field_types)

    for file in sorted_files:
        logger.info(f"Convert {file} to {osf_out_name} ")
        data_source = open_source(file, sensor_idx=0)
        last_ts = write_osf_from_source(
            data_source, osf_writer, ts_diff, last_ts)

    osf_writer.close()


if __name__ == "__main__":
    if len(sys.argv) != 3:
        logger.error("Usage: python3 pcaps_to_osf.py PCAP_Files_PATH/ PAHT/out_filename\nExit")
        exit(1)

    files_path = Path(sys.argv[1])
    osf_out_name = sys.argv[2]
    logger.info(f"pcap files path {sys.argv[1]}")
    logger.info(f"output osf filename {sys.argv[2]}")
    # pcaps_to_osf(files_path, osf_out_name)
    pcaps_to_osfs(files_path, osf_out_name)
