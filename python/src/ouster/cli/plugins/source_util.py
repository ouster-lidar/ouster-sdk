from enum import IntEnum
from functools import wraps
from ouster.sdk.client import LidarScan, ScanSource
from typing import (Callable, List, Any, Union,
                    Dict, Optional, Iterator)
from threading import Event
from dataclasses import dataclass
import queue
import click


class SourceCommandType(IntEnum):
    MULTICOMMAND_UNSUPPORTED = 0
    PROCESSOR = 1
    CONSUMER = 2


@dataclass(init=False)
class SourceCommandContext:
    source_uri: Optional[str]
    scan_source: Optional[Union[ScanSource, Any]]
    scan_iter: Optional[Iterator[LidarScan]]
    terminate_evt: Optional[Event]
    main_thread_fn: Optional[Callable[[None], None]]
    thread_fns: List[Callable[[None], None]]
    invoked_command_names: List[str]
    misc: Dict[Any, Any]
    terminate_exception: Optional[Exception]

    def __init__(self) -> None:
        self.source_uri = ""
        self.scan_source = None
        self.scan_iter = None
        self.terminate_evt = None
        self.main_thread_fn = None
        self.thread_fns = []
        self.invoked_command_names = []
        self.misc = {}
        self.terminate_exception = None

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
    type: SourceCommandType


def source_multicommand(type: SourceCommandType = SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                        retrieve_click_context: bool = False):
    def source_multicommand_wrapper(fn):
        @wraps(fn)
        def callback_wrapped(click_ctx: click.core.Context, *args, **kwargs):
            # Extract ctx.obj: SourceCommandContext from click context
            if not retrieve_click_context:
                return SourceCommandCallback(lambda ctx: fn(ctx, *args, **kwargs), type)  # type: ignore
            else:
                return SourceCommandCallback(
                        lambda ctx: fn(ctx, click_ctx, *args, **kwargs), type)  # type: ignore
        return callback_wrapped
    return source_multicommand_wrapper


class CoupledTee:
    sentinel = object()
    _queues: List[queue.Queue]

    def __init__(self, iter: Iterator, n: int = 2,
                 terminate: Optional[Event] = None,
                 copy_fn: Optional[Callable] = None,
                 poll_wait_sec: float = 0.25) -> None:
        self._iter = iter
        self._n = n
        self._queues = [queue.Queue() for _ in range(n - 1)]
        self._next = None
        self._terminate = terminate
        self._copy_fn = (copy_fn if copy_fn is not None else
            lambda x: x)
        self._poll_wait_sec = poll_wait_sec

    def main_tee(self) -> Iterator:
        try:
            for val in self._iter:
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
                        return
                    raise val
                yield val
            except queue.Empty:
                if self._terminate and self._terminate.is_set():
                    return

    @staticmethod
    def tee(iter: Iterator, n: int = 2, **kwargs) -> List[Iterator]:
        ct = CoupledTee(iter, **kwargs)
        tees = []
        tees.append(ct.main_tee())
        for i in range(n - 1):
            tees.append(ct.sub_tee(i))
        return tees


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


ROS_MODULES_ERROR_MSG = """
Error: {err_msg}

Please verify that ROS Python modules are available.

The best option is to try to install unofficial rospy packages that work
with Python 3.8 on Ubuntu 18.04/20.04 and Debian 10 without ROS:

    pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf2_ros

NOTE: If during the attempt to run the above command you get an error:

    EnvironmentError: 404 Client Error: Not Found for url: https://pypi.org/simple/rospy/

Please check installed `pip` version (20.0+ works well with extra indexes), and
if needed upgrade `pip` with (in a sourced venv):

    pip install pip -U

Alternatively, the bagpy package might work on some systems:

    pip install bagpy

Some users have even more packages missing so they may need to aditionally install:

    pip install PyYAML pycryptodome pycryptodomex

"""


def import_rosbag_modules(raise_on_fail: bool = False) -> bool:
    try:
        import rosbag  # type: ignore # noqa: F401
        import rospy  # type: ignore # noqa: F401
        import genpy  # type: ignore # noqa: F401
    except ImportError as err:
        if raise_on_fail:
            raise ModuleNotFoundError(
                ROS_MODULES_ERROR_MSG.format(err_msg=str(err)))
        return False
    return True
