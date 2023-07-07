import socket
import os
from enum import Enum
from typing import Optional


class OusterIoType(Enum):
    SENSOR = 1
    PCAP = 2
    OSF = 3
    ROSBAG = 4
    CSV = 5
    PLY = 6
    PCD = 7
    LAS = 8

    def __str__(self):
        if self.value == 1:
            return "SENSOR"
        if self.value == 2:
            return "PCAP"
        if self.value == 3:
            return "OSF"
        if self.value == 4:
            return "ROSBAG"
        if self.value == 5:
            return "CSV"
        if self.value == 6:
            return "PLY"
        if self.value == 7:
            return "PCD"
        if self.value == 8:
            return "LAS"
        return "UNKNOWN"


def extension_from_io_type(source: OusterIoType) -> Optional[str]:
    """Return a file extension for the given source type, if it's a file-based source."""
    if source == OusterIoType.PCAP:
        return '.pcap'
    elif source == OusterIoType.OSF:
        return '.osf'
    elif source == OusterIoType.ROSBAG:
        return '.bag'
    elif source == OusterIoType.CSV:
        return '.csv'
    elif source == OusterIoType.PLY:
        return '.ply'
    elif source == OusterIoType.PCD:
        return '.pcd'
    elif source == OusterIoType.LAS:
        return '.las'
    return None


def io_type_from_extension(source: str) -> OusterIoType:
    """Return an OusterIoType given the file extension for the provided file path"""
    source_lower = source.lower()
    if source_lower.endswith('.pcap'):
        return OusterIoType.PCAP
    elif source_lower.endswith('.osf'):
        return OusterIoType.OSF
    elif source_lower.endswith('.bag'):
        return OusterIoType.ROSBAG
    elif source_lower.endswith('.csv'):
        return OusterIoType.CSV
    elif source_lower.endswith('.ply'):
        return OusterIoType.PLY
    elif source_lower.endswith('.pcd'):
        return OusterIoType.PCD
    elif source_lower.endswith('.las'):
        return OusterIoType.LAS
    else:
        raise ValueError('Expecting .pcap, .osf, .bag, .ply, .pcd, .las or .csv.')


def io_type_from_magic(source: str) -> Optional[OusterIoType]:
    """Try to return an OusterIoType given a file path, using python-magic"""
    try:
        import magic
        # Note - python-magic doesn't know what .osf or .bag files are.
        type_wizard = magic.from_file(os.path.realpath(source))
        if "pcap capture file" in type_wizard:
            return OusterIoType.PCAP
        elif "Point Cloud Data" in type_wizard:
            return OusterIoType.PCD
        elif "LIDAR point data records" in type_wizard:
            return OusterIoType.LAS
        elif "CSV text" in type_wizard:
            return OusterIoType.CSV

    except ImportError:
        pass
    return None


def io_type(source: str) -> OusterIoType:
    """Return a OusterIoType given a source arg str"""
    if os.path.isfile(source):
        magic_type = io_type_from_magic(source)
        if magic_type:
            return magic_type
        io_type = io_type_from_extension(source)
        return io_type
    try:
        if socket.gethostbyname(source):
            return OusterIoType.SENSOR
    except Exception:
        pass

    raise ValueError("Source type expected to be a sensor hostname, ip address,"
                     " or a .pcap, .osf, or .bag file.")
