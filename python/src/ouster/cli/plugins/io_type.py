import socket
import os
from enum import Enum, auto
from typing import Optional


class OusterIoType(Enum):
    SENSOR = auto()
    PCAP = auto()
    OSF = auto()
    BAG = auto()
    CSV = auto()
    PLY = auto()
    PCD = auto()
    LAS = auto()

    @staticmethod
    def io_type_2_extension() -> dict:
        return {
            OusterIoType.PCAP: ".pcap",
            OusterIoType.OSF: ".osf",
            OusterIoType.BAG: ".bag",
            OusterIoType.CSV: ".csv",
            OusterIoType.PLY: ".ply",
            OusterIoType.PCD: ".pcd",
            OusterIoType.LAS: ".las"
        }

    @staticmethod
    def extension_2_io_type() -> dict:
        return {value: key for key, value in OusterIoType.io_type_2_extension().items()}


def extension_from_io_type(source: OusterIoType) -> Optional[str]:
    """Return a file extension for the given source type, if it's a file-based source."""
    return OusterIoType.io_type_2_extension().get(source)


def io_type_from_extension(source: str) -> OusterIoType:
    """Return an OusterIoType given the file extension for the provided file path"""
    ext = os.path.splitext(source)[1]
    try:
        return OusterIoType.extension_2_io_type()[ext.lower()]
    except KeyError:
        raise ValueError("Expecting", list(
            OusterIoType.extension_2_io_type().keys()))


def io_type(source: str) -> OusterIoType:
    """Return a OusterIoType given a source arg str"""
    if os.path.isfile(source):
        return io_type_from_extension(source)
    try:
        if socket.gethostbyname(source):
            return OusterIoType.SENSOR
    except Exception:
        pass

    raise ValueError("Source type expected to be a sensor hostname, ip address,"
                     " or a .pcap, .osf, or .bag file.")
