"""Sensor data visualization tools.

Visualize lidar data using OpenGL.
"""
from collections import (defaultdict, deque)
from functools import partial
import threading
import time
from typing import (Callable, ClassVar, Deque, Dict, Generic, Iterable, List,
                    Optional, Tuple, TypeVar)
import weakref

import numpy as np

from .. import client
from ..client import (_utils, ChanField)
from ..client._client import Version
from ._viz import (PointViz, Cloud, Image, Cuboid, Label, WindowCtx, Camera,
                   TargetDisplay, add_default_controls, calref_palette,
                   spezia_palette)


class LidarScanViz:
    """Visualize LidarScan data.

    Uses the supplied PointViz instance to easily display the contents of a
    LidarScan. Sets up key bindings to toggle which channel fields and returns
    are displayed, and change 2D image and point size.
    """

    @staticmethod
    def _reflectivity_pp(
            info: client.SensorInfo) -> Callable[[np.ndarray], None]:

        if Version.from_string(info.fw_rev) >= Version.from_string("v2.1.0"):

            def proc_cal(refl, update_state: bool = True) -> None:
                refl /= 255.0

            return proc_cal
        else:
            return _utils.AutoExposure()

    @staticmethod
    def _near_ir_pp(info: client.SensorInfo) -> Callable[[np.ndarray], None]:
        buc = _utils.BeamUniformityCorrector()
        ae = _utils.AutoExposure()

        def proc(ambient) -> None:
            destag = client.destagger(info, ambient)
            buc(destag)
            ambient[:] = client.destagger(info, destag, inverse=True)
            ae(ambient)

        return proc

    _cloud_mode_channels: ClassVar[List[Tuple[ChanField, ChanField]]] = [
        (ChanField.RANGE, ChanField.RANGE2),
        (ChanField.SIGNAL, ChanField.SIGNAL2),
        (ChanField.REFLECTIVITY, ChanField.REFLECTIVITY2),
        (ChanField.NEAR_IR, ChanField.NEAR_IR),
    ]

    _available_fields: List[ChanField]
    _cloud_palette: Optional[np.ndarray]
    _field_pp: Dict[ChanField, Callable[[np.ndarray], None]]

    def __init__(self, viz: PointViz, meta: client.SensorInfo) -> None:
        """
        Args:
            viz: the PointViz instance to use
            meta: sensor metadata used to interpret scans
        """

        self._lock = threading.Lock()
        self._available_fields = []

        # cloud display state
        self._cloud_mode_ind = 0  # index into _cloud_mode_channels
        self._cloud_enabled = [True, True]
        self._cloud_pt_size = 2.0
        self._cloud_palette = spezia_palette

        # image display state
        self._img_ind = [0, 1]
        self._img_size_fraction = 6
        self._img_aspect = (meta.beam_altitude_angles[0] -
                            meta.beam_altitude_angles[-1]) / 360.0

        # misc display state
        self._ring_size = 1

        # TODO handle second return (sharing procs)
        range_pp = _utils.AutoExposure()
        signal_pp = _utils.AutoExposure()
        refl_pp = LidarScanViz._reflectivity_pp(meta)
        nearir_pp = LidarScanViz._near_ir_pp(meta)

        self._field_pp = {
            ChanField.RANGE: range_pp,
            ChanField.RANGE2: partial(range_pp, update_state=False),
            ChanField.SIGNAL: signal_pp,
            ChanField.SIGNAL2: partial(signal_pp, update_state=False),
            ChanField.REFLECTIVITY: refl_pp,
            ChanField.REFLECTIVITY2: partial(refl_pp, update_state=False),
            ChanField.NEAR_IR: nearir_pp,
        }

        self._viz = viz

        self._metadata = meta
        self._clouds = (Cloud(meta), Cloud(meta))
        self._viz.add(self._clouds[0])
        self._viz.add(self._clouds[1])

        self._images = (Image(), Image())
        self._viz.add(self._images[0])
        self._viz.add(self._images[1])

        # initialize image sizes
        self.update_image_size(0)

        # initialize rings
        self._viz.target_display.set_ring_size(self._ring_size)
        self._viz.target_display.enable_rings(True)

        # initialize osd
        self._osd = Label("", 0, 1)
        self._viz.add(self._osd)
        self._osd_enabled = True

        # key bindings. will be called from rendering thread, must be synchronized
        key_bindings: Dict[Tuple[int, int], Callable[[LidarScanViz], None]] = {
            (ord('E'), 0): partial(LidarScanViz.update_image_size, amount=1),
            (ord('E'), 1): partial(LidarScanViz.update_image_size, amount=-1),
            (ord('P'), 0): partial(LidarScanViz.update_point_size, amount=1),
            (ord('P'), 1): partial(LidarScanViz.update_point_size, amount=-1),
            (ord('1'), 0): partial(LidarScanViz.toggle_cloud, i=0),
            (ord('2'), 0): partial(LidarScanViz.toggle_cloud, i=1),
            (ord('B'), 0): partial(LidarScanViz.cycle_img_mode, i=0),
            (ord('N'), 0): partial(LidarScanViz.cycle_img_mode, i=1),
            (ord('M'), 0): LidarScanViz.cycle_cloud_mode,
            (ord("'"), 0): partial(LidarScanViz.update_ring_size, amount=1),
            (ord("'"), 1): partial(LidarScanViz.update_ring_size, amount=-1),
            (ord("O"), 0): LidarScanViz.toggle_osd,
        }

        # take care to avoid a reference cycle; holding onto self in the callback
        # passed to native code would cause a memory leak
        weakself = weakref.ref(self)

        def handle_keys(ctx: WindowCtx, key: int, mods: int) -> bool:
            self = weakself()
            if self is not None and (key, mods) in key_bindings:
                key_bindings[key, mods](self)
                # render new frame
                self.draw()
            return True

        self._viz.push_key_handler(handle_keys)
        add_default_controls(self._viz)

    def cycle_img_mode(self, i: int) -> None:
        """Change the displayed field of the i'th image."""
        with self._lock:
            nfields = len(self._available_fields)
            if nfields > 0:
                self._img_ind[i] = (self._img_ind[i] + 1) % nfields

    def cycle_cloud_mode(self) -> None:
        """Change the channel field used to color the 3D point cloud."""
        with self._lock:
            nfields = len(LidarScanViz._cloud_mode_channels)
            self._cloud_mode_ind = (self._cloud_mode_ind + 1) % nfields
            new_fields = LidarScanViz._cloud_mode_channels[
                self._cloud_mode_ind]
            self._cloud_palette = (calref_palette if ChanField.REFLECTIVITY
                                   in new_fields else spezia_palette)

    def toggle_cloud(self, i: int) -> None:
        """Toggle whether the i'th return is displayed."""
        with self._lock:
            if self._cloud_enabled[i]:
                self._cloud_enabled[i] = False
                self._viz.remove(self._clouds[i])
            else:
                self._cloud_enabled[i] = True
                self._viz.add(self._clouds[i])

    def update_point_size(self, amount: int) -> None:
        """Change the point size of the 3D cloud."""
        with self._lock:
            self._cloud_pt_size = min(10.0,
                                      max(1.0, self._cloud_pt_size + amount))
            for cloud in self._clouds:
                cloud.set_point_size(self._cloud_pt_size)

    def update_image_size(self, amount: int = 1) -> None:
        """Change the size of the 2D image."""
        with self._lock:
            size_fraction_max = 20
            self._img_size_fraction = (self._img_size_fraction + amount +
                                       (size_fraction_max + 1)) % (
                                           size_fraction_max + 1)

            vfrac = self._img_size_fraction / size_fraction_max
            hfrac = vfrac / 2 / self._img_aspect

            self._images[0].set_position(-hfrac, hfrac, 1 - vfrac, 1)
            self._images[1].set_position(-hfrac, hfrac, 1 - vfrac * 2,
                                         1 - vfrac)

            # center camera target in area not taken up by image
            self._viz.camera.set_proj_offset(0, vfrac)

    def update_ring_size(self, amount: int = 1) -> None:
        """Change distance ring size."""
        with self._lock:
            self._ring_size = min(2, max(-2, self._ring_size + amount))
            self._viz.target_display.set_ring_size(self._ring_size)

    def toggle_osd(self, state: Optional[bool] = None) -> None:
        """Show or hide the on-screen display."""
        with self._lock:
            self._osd_enabled = not self._osd_enabled if state is None else state

    @property
    def scan(self) -> client.LidarScan:
        """The currently displayed scan."""
        with self._lock:
            return self._scan

    @scan.setter
    def scan(self, scan: client.LidarScan) -> None:
        """Set the scan to display"""
        with self._lock:
            self._scan = scan

    def draw(self, update: bool = True) -> bool:
        """Process and draw the latest state to the screen."""
        with self._lock:
            self._draw()

        if update:
            return self._viz.update()
        else:
            return False

    # i/o and processing, called from client thread
    # usually need to synchronize with key handlers, which run in render thread
    def _draw(self) -> None:

        # figure out what to draw based on current viz state. Need to
        scan = self._scan
        self._available_fields = list(scan.fields)

        img_fields = (self._available_fields[self._img_ind[0]],
                      self._available_fields[self._img_ind[1]])
        cloud_fields = LidarScanViz._cloud_mode_channels[self._cloud_mode_ind]

        # changed in handler when displaying cal ref
        palette = self._cloud_palette
        self._cloud_palette = None

        # extract field data
        field_data: Dict[ChanField, np.ndarray]
        field_data = defaultdict(lambda: np.zeros(
            (scan.h, scan.w), dtype=np.double))

        for field in {*img_fields, *cloud_fields}:
            if field in scan.fields:
                field_data[field] = scan.field(field).astype(np.double)

        # apply post-processing
        for field, data in field_data.items():
            if field in self._field_pp:
                self._field_pp[field](data)

        # update 3d display
        for i, range_field in ((0, ChanField.RANGE), (1, ChanField.RANGE2)):
            if range_field in scan.fields:
                self._clouds[i].set_range(scan.field(range_field))
            else:
                self._clouds[i].set_range(
                    np.zeros((scan.h, scan.w), dtype=np.uint32))

            self._clouds[i].set_key(field_data[cloud_fields[i]])
            if palette is not None:
                self._clouds[i].set_palette(palette)

        # update 2d images
        for i in (0, 1):
            self._images[i].set_image(
                client.destagger(self._metadata,
                                 field_data[img_fields[i]].astype(np.float32)))

        meta = self._metadata
        enable_ind = [i + 1 for i, b in enumerate(self._cloud_enabled) if b]
        if self._osd_enabled:
            self._osd.set_text(
                f"image: {img_fields[0]}/{img_fields[1]}\n"
                f"cloud{enable_ind}: {cloud_fields[0]}\n"
                f"frame: {scan.frame_id}\n"
                f"sensor ts: {scan.timestamp[0] / 1e9:.3f}s\n"
                f"profile: {str(meta.format.udp_profile_lidar)}\n"
                f"{meta.prod_line} {meta.fw_rev} {meta.mode}")
        else:
            self._osd.set_text("")


T = TypeVar('T')


class _Seekable(Generic[T]):
    """Wrap an iterable to support seeking by index.

    Similar to `more_itertools.seekable` but keeps indexes stable even values
    are evicted from the cache.

    The :meth:`seek` and :meth:`__next__` methods maintain the invariant:

        (read_ind - len(cache)) < next_ind <= read_ind + 1
    """

    def __init__(self, it: Iterable[T], maxlen=50) -> None:
        self._next_ind = 0  # index of next value to be returned
        self._read_ind = -1  # index of most recent (leftmost) value in cache
        self._iterable = it
        self._it = iter(it)
        self._maxlen = maxlen
        self._cache: Deque[T] = deque([], maxlen)

    def __iter__(self) -> '_Seekable[T]':
        return self

    def __next__(self) -> T:
        # next value already read, is in cache
        if self._next_ind <= self._read_ind:
            t = self._cache[self._read_ind - self._next_ind]
            self._next_ind += 1
            return t
        # next value comes from iterator
        elif self._next_ind == self._read_ind + 1:
            t = next(self._it)
            self._cache.appendleft(t)
            if len(self._cache) > self._maxlen:
                self._cache.pop()
            self._next_ind += 1
            self._read_ind += 1
            return t
        else:
            raise AssertionError("Violated: next_ind <= read_ind + 1")

    @property
    def next_ind(self) -> int:
        return self._next_ind

    def seek(self, ind: int) -> bool:
        """Update iterator position to index `ind`.

        Args:
            ind: the desired index to be read on the subsequent call to
                 :meth:`__next__`

        Returns:
            True if seeking succeeded, False otherwise. Seeking may fail if the
            desired index has been evicted from the cache.

        Raises:
            StopIteration if seeking beyond the end of the iterator
        """

        # seek forward until ind is next to be read
        while ind > self._next_ind:
            next(self)

        # here ind <= _read_ind + 1. Left to check whether value is in the cache
        if ind > (self._read_ind - len(self._cache)):
            self._next_ind = ind
            return True
        else:
            # value not in cache, seek failed
            return False

    def close(self) -> None:
        """Close the underlying iterable, if supported."""
        if hasattr(self._iterable, 'close'):
            self._iterable.close()  # type: ignore


class SimpleViz:
    """Visualize a stream of LidarScans.

    Handles controls for playback speed, pausing and stepping."""

    _buflen: ClassVar[int] = 50

    _playback_rates: ClassVar[Tuple[float, ...]]
    _playback_rates = (0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 0.0)

    def __init__(self,
                 scans: client.ScanSource,
                 rate: Optional[float] = 1.0) -> None:
        """
        Args:
            scans: The stream of scans to visualze.
            rate: Playback rate. One of 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0 or
                  None for "live" playback.

        Raises:
            ValueError: if the specified rate isn't one of the options
        """
        self._viz = PointViz("Ouster Viz")
        self._scan_viz = LidarScanViz(self._viz, scans.metadata)

        self._lock = threading.Lock()
        self._scans = _Seekable(scans, maxlen=SimpleViz._buflen)
        self._metadata = scans.metadata
        self._live = (rate is None)
        self._rate_ind = SimpleViz._playback_rates.index(rate or 0.0)

        # pausing and stepping
        self._cv = threading.Condition()
        self._paused = False
        self._step = 0
        self._proc_exit = False

        # playback status display
        self._playback_osd = Label("", 1, 1, align_right=True)
        self._viz.add(self._playback_osd)
        self._osd_enabled = True
        self._update_playback_osd()

        key_bindings: Dict[Tuple[int, int], Callable[[SimpleViz], None]] = {
            (ord(','), 0): partial(SimpleViz.seek_relative, n_frames=-1),
            (ord(','), 2): partial(SimpleViz.seek_relative, n_frames=-10),
            (ord('.'), 0): partial(SimpleViz.seek_relative, n_frames=1),
            (ord('.'), 2): partial(SimpleViz.seek_relative, n_frames=10),
            (ord(' '), 0): SimpleViz.toggle_pause,
            (ord('O'), 0): SimpleViz.toggle_osd,
        }

        # only allow changing rate when not in "live" mode
        if not self._live:
            key_bindings.update({
                (ord(','), 1):
                partial(SimpleViz.modify_rate, amount=-1),
                (ord('.'), 1):
                partial(SimpleViz.modify_rate, amount=1),
            })

        weakself = weakref.ref(self)

        def handle_keys(ctx: WindowCtx, key: int, mods: int) -> bool:
            self = weakself()
            if self is not None and (key, mods) in key_bindings:
                key_bindings[key, mods](self)
                # override rather than add bindings
                return False
            return True

        self._viz.push_key_handler(handle_keys)

    def _update_playback_osd(self) -> None:
        if not self._osd_enabled:
            self._playback_osd.set_text("")
        elif self._paused:
            self._playback_osd.set_text("playback: paused")
        elif self._live:
            self._playback_osd.set_text("playback: live")
        else:
            rate = SimpleViz._playback_rates[self._rate_ind]
            self._playback_osd.set_text(
                f"playback: {str(rate) + 'x' if rate else  'max'}")

    def toggle_pause(self) -> None:
        """Pause or unpause the visualization."""
        with self._cv:
            self._paused = not self._paused
            self._update_playback_osd()
            if not self._paused:
                self._cv.notify()

    def seek_relative(self, n_frames: int) -> None:
        """Seek forward of backwards in the stream."""
        with self._cv:
            self._paused = True
            self._step = n_frames
            self._update_playback_osd()
            self._cv.notify()

    def modify_rate(self, amount: int) -> None:
        """Switch between preset playback rates."""
        n_rates = len(SimpleViz._playback_rates)
        with self._cv:
            self._rate_ind = max(0, min(n_rates - 1, self._rate_ind + amount))
            self._update_playback_osd()

    def toggle_osd(self, state: Optional[bool] = None) -> None:
        """Show or hide the on-screen display."""
        with self._cv:
            self._osd_enabled = not self._osd_enabled if state is None else state
            self._scan_viz.toggle_osd(self._osd_enabled)
            self._update_playback_osd()
            self._scan_viz.draw()

    def _frame_period(self) -> float:
        rate = SimpleViz._playback_rates[self._rate_ind]
        if rate and not self._paused:
            return 1.0 / (self._metadata.mode.frequency * rate)
        else:
            return 0.0

    def _process(self) -> None:

        last_ts = time.monotonic()
        try:
            while True:
                # wait until unpaused, step, or quit
                with self._cv:
                    self._cv.wait_for(lambda: not self._paused or self._step or
                                      self._proc_exit)
                    if self._proc_exit:
                        break
                    if self._step:
                        seek_ind = self._scans.next_ind + self._step - 1
                        self._step = 0
                        if not (self._scans.seek(seek_ind)):
                            continue
                    period = self._frame_period()

                # process new data
                self._scan_viz.scan = next(self._scans)
                self._scan_viz.draw(update=False)

                # sleep for remainder of scan period
                to_sleep = max(0.0, period - (time.monotonic() - last_ts))
                time.sleep(to_sleep)
                last_ts = time.monotonic()

                # show new data
                self._viz.update()

        except StopIteration:
            pass

        finally:
            # signal rendering (main) thread to exit
            self._viz.running(False)

    def run(self) -> None:
        """Start reading scans and visualizing the stream.

        Must be called from the main thread on macos. Will close the provided
        scan source before returning.

        Returns:
            When the stream is consumed or the visualizer window is closed.
        """
        try:
            print("Starting processing thread...")
            self._proc_exit = False
            proc_thread = threading.Thread(target=self._process, name="Proc")
            proc_thread.start()

            print("Starting rendering loop...")
            self._viz.run()
            print("Done rendering loop")
        except KeyboardInterrupt:
            pass
        finally:
            # some scan sources may be waiting on IO, blocking the processing thread
            self._scans.close()

            # processing thread will still be running if e.g. viz window was closed
            with self._cv:
                self._proc_exit = True
                self._cv.notify()

            print("Joining processing thread")
            proc_thread.join()


__all__ = [
    'PointViz', 'Cloud', 'Image', 'Cuboid', 'Label', 'WindowCtx', 'Camera',
    'TargetDisplay', 'add_default_controls', 'calref_palette', 'spezia_palette'
]
