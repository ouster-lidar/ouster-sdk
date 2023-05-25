from typing import Callable, Iterable, Iterator, TypeVar

from more_itertools import consume

from ouster.client import Packet, LidarPacket

T = TypeVar('T')


def ichunked_before(it: Iterable[T],
                    pred: Callable[[T], bool]) -> Iterator[Iterator[T]]:
    """Return the given stream chunked by the predicate.

    Each sub-iterator will be fully consumed when the next chunk is
    requested. No caching of unused items is performed, so client code should
    evaluate sub-iterators (e.g. into lists) to avoid dropping items.

    This should behave same as more_itertools.split_before, except that chunks
    aren't eagerly evaluated into lists. This makes it safe to use on streams
    where it's possible that ``pred`` never evaluates to true.
    """
    i = iter(it)

    # flag used by chunks to signal that the underlying iterator is exhausted
    done = False

    # first item of the next chunk. See: nonlocal below
    try:
        t = next(i)
    except StopIteration:
        return

    def chunk() -> Iterator[T]:
        nonlocal done, t

        yield t
        for t in i:
            if pred(t):
                break
            else:
                yield t
        # only if the iterator is exhausted
        else:
            done = True

    while not done:
        c = chunk()
        yield c
        consume(c)


def ichunked_framed(
    packets: Iterable[Packet],
    pred: Callable[[Packet],
                   bool] = lambda _: True) -> Iterator[Iterator[Packet]]:
    """Delimit a packets when the frame id changes and pred is true."""

    last_f_id = -1

    def frame_boundary(p: Packet) -> bool:
        nonlocal last_f_id
        if isinstance(p, LidarPacket):
            f_id = p.frame_id
            changed = last_f_id != -1 and f_id != last_f_id
            last_f_id = f_id
            return changed and pred(p)
        return False

    return ichunked_before(packets, frame_boundary)


def n_frames(packets: Iterable[Packet], n: int) -> Iterator[Packet]:
    for i, frame in enumerate(ichunked_framed(packets)):
        if i < n:
            yield from frame
        else:
            break
