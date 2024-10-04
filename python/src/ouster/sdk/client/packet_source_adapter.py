from typing import Iterator

from ouster.sdk._bindings.client import SensorInfo, Packet
from .core import PacketSource
from .multi import PacketMultiSource


class PacketSourceAdapter(PacketSource):
    """Represents only data from one sensor stream."""

    def __init__(self, packet_source: PacketMultiSource, stream_idx: int = 0) -> None:
        self._source = packet_source
        self._stream_idx = stream_idx

    @property
    def metadata(self) -> SensorInfo:
        """Metadata objects associated with the packet stream."""
        return self._source.metadata[self._stream_idx]

    @property
    def is_live(self) -> bool:
        """True if data obtained from the RUNNING sensor or as a stream from the socket

        Returns:
            True if data obtained from the RUNNING sensor or as a stream from the socket
            False if data is read from a stored media. Restarting an ``iter()`` means that
                    the data can be read again.
        """
        return self._source.is_live

    def __iter__(self) -> Iterator[Packet]:

        def _stream_iter(source: PacketMultiSource) -> Iterator[Packet]:
            for idx, p in source:
                if idx == self._stream_idx:
                    yield p

        return _stream_iter(self._source)

    def close(self) -> None:
        """Release the underlying resource, if any."""
        pass

    def __del__(self) -> None:
        """Automatic release of any underlying resource."""
        self.close()
