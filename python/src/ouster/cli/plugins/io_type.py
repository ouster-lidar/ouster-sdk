import socket
import os
from enum import Enum
from typing import Optional


class OusterIoType(Enum):
    SENSOR = 1
    PCAP = 2
    OSF = 3
    ROSBAG = 4


def extension_from_io_type(source: OusterIoType) -> Optional[str]:
    """Return a file extension for the given source type, if it's a file-based source."""
    if source == OusterIoType.PCAP:
        return '.pcap'
    elif source == OusterIoType.OSF:
        return '.osf'
    elif source == OusterIoType.ROSBAG:
        return '.bag'
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
    else:
        raise ValueError('Expecing .pcap, .osf, or .bag.')


def io_type_from_magic(source: str) -> Optional[OusterIoType]:
    """Try to return an OusterIoType given a file path, using python-magic"""
    try:
        import magic
        # Note - python-magic doesn't know what .osf or .bag files are.
        if magic.from_file(source, mime=True) == 'application/vnd.tcpdump.pcap':
            return OusterIoType.PCAP
    except ImportError:
        pass
    return None


def io_type(source: str) -> OusterIoType:
    """Return a OusterIoType given a source arg str"""
    if os.path.isfile(source):
        magic_type = io_type_from_magic(source)
        if magic_type:
            return magic_type
        return io_type_from_extension(source)
    try:
        if socket.gethostbyname(source):
            return OusterIoType.SENSOR
    except Exception:
        pass

    raise ValueError(
        "Source type expected to be a sensor hostname, ip address, or a .pcap, .osf, or .bag file.")
