from enum import IntEnum
from functools import wraps
from ouster.sdk.core import LidarScan, ScanSource
from typing import (Callable, List, Any, Union, Tuple,
                    Dict, Optional, Iterator, Iterable)
from threading import Event
from dataclasses import dataclass
from ouster.sdk.sensor import ClientTimeout
import queue
import click

from datetime import datetime


class SourceCommandType(IntEnum):
    MULTICOMMAND_UNSUPPORTED = 0
    PROCESSOR_UNREPEATABLE = 1
    PROCESSOR = 2
    CONSUMER = 3


@dataclass(init=False)
class SourceCommandContext:
    source_uri: Optional[str]
    source_options: Dict[str, Any]
    other_options: Dict[str, Any]
    scan_source: Optional[Union[ScanSource, Any]]
    scan_iter: Optional[Iterable[List[Optional[LidarScan]]]]
    terminate_evt: Optional[Event]
    main_thread_fn: Optional[Callable[[None], None]]
    invoked_command_names: List[str]
    misc: Dict[Any, Any]
    terminate_exception: Optional[Exception]

    def __init__(self) -> None:
        self.source_uri = ""
        self.scan_source = None
        self.scan_iter = None
        self.terminate_evt = None
        self.main_thread_fn = None
        self.invoked_command_names = []
        self.misc = {}
        self.terminate_exception = None
        self.source_options = {}
        self.other_options = {}

    # [kk] NOTE: get and __getitem__ are defined to support
    # older code that still treats ctx.obj as a dict
    # We should refactor those calls out, and remove these methods
    def get(self, key: str, default: Any) -> Any:
        return self.misc.get(key, default)

    def __getitem__(self, key: str) -> Any:
        return self.misc[key]


@dataclass
class SourceCommandCallback:
    callback_fn: Callable[[SourceCommandContext], None]
    prerun_fn: Callable[[SourceCommandContext], None]
    type: SourceCommandType


def source_multicommand(type: SourceCommandType = SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                        retrieve_click_context: bool = False,
                        prerun: Optional[Callable[[SourceCommandContext], None]] = None):
    def source_multicommand_wrapper(fn):
        @wraps(fn)
        def callback_wrapped(click_ctx: click.core.Context, *args, **kwargs):
            prefn = None
            if prerun is not None:
                prefn = lambda ctx: prerun(ctx, *args, **kwargs)
            # Extract ctx.obj: SourceCommandContext from click context
            if not retrieve_click_context:
                return SourceCommandCallback(lambda ctx: fn(ctx, *args, **kwargs), prefn, type)  # type: ignore
            else:
                return SourceCommandCallback(
                        lambda ctx: fn(ctx, click_ctx, *args, **kwargs), prefn, type)  # type: ignore
        return callback_wrapped
    return source_multicommand_wrapper


class CoupledTee:
    sentinel = object()
    _queues: List[queue.Queue]

    def __init__(self, iter: Iterator, n: int = 2,
                 terminate: Optional[Event] = None,
                 copy_fn: Optional[Callable] = None,
                 poll_wait_sec: float = 0.25,
                 loop: bool = False) -> None:
        self._iter = iter
        self._n = n
        self._queues = [queue.Queue() for _ in range(n - 1)]
        self._next = None
        self._terminate = terminate
        self._copy_fn = (copy_fn if copy_fn is not None else
            lambda x: x)
        self._poll_wait_sec = poll_wait_sec
        self._loop = loop

    def main_tee(self) -> Iterator:
        try:
            # type ignored because generators are tricky to mypy
            for val in self._iter():  # type: ignore
                for q in self._queues:
                    q.put(val)
                yield val
                for q in self._queues:
                    with q.all_tasks_done:
                        while q.unfinished_tasks:
                            q.all_tasks_done.wait(self._poll_wait_sec)
                            if self._terminate and self._terminate.is_set():
                                return
        except Exception as ex:
            for q in self._queues:
                q.put(ex)
            raise ex

        # propagate StopIteration to sub_tees
        for q in self._queues:
            q.put(StopIteration())

    def sub_tee(self, idx: int) -> Iterator:
        while True:
            try:
                val = self._queues[idx].get(block=True, timeout=0.5)
                self._queues[idx].task_done()
                if isinstance(val, Exception):
                    # python doesnt allow you to throw a stop iteration here
                    if isinstance(val, StopIteration):
                        if self._loop:
                            yield []
                            continue
                        return
                    if isinstance(val, ClientTimeout):
                        return
                    raise val
                yield val
            except queue.Empty:
                if self._terminate and self._terminate.is_set():
                    return

    @staticmethod
    def tee(iter: Iterator, n: int = 2, **kwargs) -> Tuple[Iterable, List[Iterator]]:
        ct = CoupledTee(iter, **kwargs)
        tees = []
        for i in range(n - 1):
            tees.append(ct.sub_tee(i))
        # type ignored because generators are tricky to mypy
        return ct.main_tee, tees  # type: ignore


def _join_with_conjunction(things_to_join: List[str], separator: str = ', ', conjunction: str = 'or') -> str:
    """Given a list of things, return a string like
    'Thing A, Thing B, or Thing C'
    """
    strings = [str(x) for x in things_to_join]
    if conjunction and len(strings) > 1:
        strings[-1] = conjunction + " " + strings[-1]
    if len(strings) == 2:
        if conjunction:
            return ' '.join(strings)
        else:
            return separator.join(strings)
    return separator.join(strings)


def _nanos_to_string(nanos):
    return datetime.utcfromtimestamp(nanos / 1e9).strftime('%Y-%m-%d %H:%M:%S.%f')
