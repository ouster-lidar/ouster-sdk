#  type: ignore
from typing import Optional, Dict, Tuple, List, Callable, Union
from enum import Enum

import os
import numpy as np
import threading
from functools import partial

from ouster.sdk import client
from ouster.sdk.client import ChanField
import ouster.sdk.viz as viz
from ouster.sdk.viz import AxisWithLabel
from ouster.sdk.util import img_aspect_ratio
from ouster.sdk.viz import (PointViz, WindowCtx, push_point_viz_handler,
                        calref_palette, spezia_palette)


class MultiLidarScanViz:
    """Multi LidarScan clouds visualizer"""

    class CloudMode(Enum):
        REFLECTIVITY = 0
        RANGE = 1
        SIGNAL = 2
        NEAR_IR = 3
        RGB = 4

    _mode_to_channels: Dict[CloudMode, List[ChanField]] = {
        CloudMode.RANGE: [ChanField.RANGE],
        CloudMode.REFLECTIVITY: [ChanField.REFLECTIVITY],
        CloudMode.SIGNAL: [ChanField.SIGNAL],
        CloudMode.NEAR_IR: [ChanField.NEAR_IR],
        CloudMode.RGB:
        [ChanField.CUSTOM0, ChanField.CUSTOM1, ChanField.CUSTOM2]
    }

    class ImagesLayout(Enum):
        HORIZONTAL = 0
        VERTICAL = 1

    def __init__(self,
                 metas: List[client.SensorInfo],
                 *,
                 source_name: str = "",
                 point_viz: Optional[PointViz] = None) -> None:

        # used to synchronize key handlers and _draw()
        self._lock = threading.Lock()

        self._source_name = source_name
        self._metas = metas
        assert len(self._metas) > 0, "ERROR: Expect at least one sensor"

        self._sensor_enabled = [True for _ in self._metas]

        self._sensor_img_aspects = [img_aspect_ratio(m) for m in self._metas]

        # initialize Auto Exposures
        self._ae_enabled = True
        self._ae_signal = [
            client._utils.AutoExposure() for _ in self._metas
        ]
        self._ae_nearir = [
            client._utils.AutoExposure() for _ in self._metas
        ]
        self._buc_nearir = [
            client._utils.BeamUniformityCorrector() for _ in self._metas
        ]
        self._ae_range = client._utils.AutoExposure()

        self._viz = point_viz or PointViz("Ouster Mutli Sensor Viz")

        # initial point size in scan clouds
        self._cloud_pt_size = 1.0

        self._clouds = []
        self._images = []
        self._image_labels = []
        for idx, m in enumerate(self._metas):
            # initialize clouds
            self._clouds.append(viz.Cloud(m))
            self._clouds[-1].set_point_size(self._cloud_pt_size)
            self._viz.add(self._clouds[-1])

            # initialize images and labels
            self._images.append(viz.Image())
            self._viz.add(self._images[-1])
            self._image_labels.append(viz.Label(m.hostname, 0.0, 0.0))
            self._viz.add(self._image_labels[-1])

        self._cloud_mode = MultiLidarScanViz.CloudMode.REFLECTIVITY
        self._cloud_palette = calref_palette

        self._images_layout = MultiLidarScanViz.ImagesLayout.HORIZONTAL

        self._ring_size = 1
        self._ring_line_width = 1

        # set initial image sizes
        self._img_size_fraction = 1

        # initialize rings
        self._viz.target_display.set_ring_size(self._ring_size)
        self._viz.target_display.enable_rings(True)

        # initialize osd
        self._osd_enabled = True
        self._osd = viz.Label("", 0, 1)
        self._viz.add(self._osd)

        # initialize scan axis helpers
        self._scan_axis_enabled = True
        self._scan_axis = []
        # sensors axis
        for idx, m in enumerate(self._metas):
            self._scan_axis.append(
                AxisWithLabel(self._viz,
                              pose=m.extrinsic,
                              label=str(idx + 1),
                              thickness=3))
        # system center axis
        self._scan_axis_origin = AxisWithLabel(self._viz,
                                               label="O",
                                               thickness=5,
                                               label_scale=0.4)

        self._scan_poses_enabled = True

        self._column_poses_identity = []
        for m in self._metas:
            self._column_poses_identity.append(
                np.array([np.eye(4) for _ in range(m.format.columns_per_frame)],
                         order='F',
                         dtype=np.float32))

        self._scan_num = -1

        # extension point for the OSD text, inserts on top of current OSD
        self._osd_text_extra: Callable[[], str] = lambda: ""

        # key bindings. will be called from rendering thread, must be synchronized
        key_bindings: Dict[Tuple[int, int], Callable[[MultiLidarScanViz], None]] = {
            (ord('H'), 0): MultiLidarScanViz.toggle_scan_axis,
            (ord('P'), 0): partial(MultiLidarScanViz.update_point_size, amount=1),
            (ord('P'), 1): partial(MultiLidarScanViz.update_point_size, amount=-1),
            (ord('M'), 0): MultiLidarScanViz.update_cloud_mode,
            (ord('L'), 0): MultiLidarScanViz.update_images_layout,
            (ord('E'), 0): partial(MultiLidarScanViz.update_image_size, amount=1),
            (ord('E'), 1): partial(MultiLidarScanViz.update_image_size, amount=-1),
            (ord('A'), 1): MultiLidarScanViz.toggle_auto_exposure,
            (ord('1'), 2): partial(MultiLidarScanViz.toggle_sensor, i=0),
            (ord('2'), 2): partial(MultiLidarScanViz.toggle_sensor, i=1),
            (ord('3'), 2): partial(MultiLidarScanViz.toggle_sensor, i=2),
            (ord('4'), 2): partial(MultiLidarScanViz.toggle_sensor, i=3),
            (ord('5'), 2): partial(MultiLidarScanViz.toggle_sensor, i=4),
            (ord('6'), 2): partial(MultiLidarScanViz.toggle_sensor, i=5),
            (ord('7'), 2): partial(MultiLidarScanViz.toggle_sensor, i=6),
            (ord('8'), 2): partial(MultiLidarScanViz.toggle_sensor, i=7),
            (ord('9'), 2): partial(MultiLidarScanViz.toggle_sensor, i=8),
            (ord("'"), 0): partial(MultiLidarScanViz.update_ring_size, amount=1),
            (ord("'"), 1): partial(MultiLidarScanViz.update_ring_size, amount=-1),
            (ord("'"), 2): MultiLidarScanViz.cicle_ring_line_width,
            (ord("O"), 0): MultiLidarScanViz.toggle_osd,
            (ord("T"), 0): MultiLidarScanViz.toggle_scan_poses,
        }

        def handle_keys(self: MultiLidarScanViz, ctx: WindowCtx, key: int,
                        mods: int) -> bool:
            if (key, mods) in key_bindings:
                draw = key_bindings[key, mods](self)
                if draw:
                    self.draw()
                else:
                    self._viz.update()
            return True

        push_point_viz_handler(self._viz, self, handle_keys)
        viz.add_default_controls(self._viz)

        self._image_size_initialized = False

    def update_image_size(self, amount: int) -> None:
        """Change the size of the 2D image and position image labels."""
        with self._lock:
            size_fraction_max = 20
            self._img_size_fraction = (self._img_size_fraction + amount +
                                       (size_fraction_max + 1)) % (
                                           size_fraction_max + 1)

            # inverted aspects exclusive prefix sum calculations
            # used for horizontal 2d images layout
            _enabled_sensor_aspects = [
                (i, a) for i, a in enumerate(self._sensor_img_aspects)
                if self._sensor_enabled[i]
            ]

            _enabled_sensor_aspects_sum = [0] * len(_enabled_sensor_aspects)
            _enabled_sensor_aspects_total = 0

            for idx_num, (idx, aspect) in enumerate(_enabled_sensor_aspects):
                if idx_num == 0:
                    _enabled_sensor_aspects_sum[idx_num] = 0
                else:
                    _enabled_sensor_aspects_sum[
                        idx_num] = _enabled_sensor_aspects_total
                _enabled_sensor_aspects_total += 1 / aspect

            # total horizontal width in img coordinates
            # NOTE: It's updating on window resize, but there are is no such
            # handler, so it's expected to have labels position off till the
            # next refresh on "E" or "O" keys
            hwidth = 2.0 * self._viz.viewport_width / self._viz.viewport_height

            # total vertical span is 2.0: from [-1.0, 1.0]
            vfrac = 2.0 * self._img_size_fraction / size_fraction_max

            _enabled_images = [
                (i, img, img_label)
                for i, (img, img_label) in enumerate(zip(self._images, self._image_labels))
                if self._sensor_enabled[i]
            ]

            # update image labels using enabled and osd_enabled states
            for idx, img_label in enumerate(self._image_labels):
                if (self._sensor_enabled[idx] and
                        self._img_size_fraction != 0 and self._osd_enabled):
                    sensor_str = self._metas[idx].hostname.replace("_", "..")
                    img_label.set_text(f"{idx + 1}: {sensor_str}")
                else:
                    img_label.set_text("")

            # placement of enabled images and labels according to image layout
            for idx_num, (idx, img, img_label) in enumerate(_enabled_images):
                hfrac = vfrac / self._sensor_img_aspects[idx]
                if self._images_layout == MultiLidarScanViz.ImagesLayout.HORIZONTAL:
                    # HORIZONTAL layout
                    x1_pos = (_enabled_sensor_aspects_sum[idx_num] -
                              _enabled_sensor_aspects_total / 2) * vfrac

                    x2_pos = x1_pos + hfrac
                    img.set_position(x1_pos, x2_pos, 1 - vfrac, 1)
                    img.set_hshift(0)
                    x1_pos_label = x1_pos
                    # converting to label coordinates
                    x1_pos_label = (x1_pos_label + hwidth / 2) / hwidth
                    img_label.set_position(x1_pos_label, 0, align_top=True)
                elif self._images_layout == MultiLidarScanViz.ImagesLayout.VERTICAL:
                    # VERTICAL layout
                    y1_pos = (idx_num + 1) * vfrac
                    y2_pos = idx_num * vfrac
                    # left alignment of the images
                    x1_pos = 0
                    x2_pos = hfrac
                    img.set_position(x1_pos, x2_pos, 1 - y1_pos,
                                     1 - y2_pos)
                    img.set_hshift(-1)
                    x1_pos_label = - hwidth / 2
                    # converting to label coordinates
                    x1_pos_label = (x1_pos_label + hwidth / 2) / hwidth  # i.e. 0
                    img_label.set_position(x1_pos_label,
                                           1 - (2 - y2_pos) / 2,
                                           align_top=True)

            self._viz.camera.set_proj_offset(0, vfrac / 2)
        return False

    @property
    def metadata(self) -> List[client.SensorInfo]:
        """Metadatas for the displayed sensors."""
        return self._metas

    @property
    def scan(self) -> Tuple[Optional[client.LidarScan]]:
        """The currently displayed scans."""
        return self._scan

    @property
    def scan_num(self) -> int:
        """The currently displayed scan number"""
        return self._scan_num

    def update(self,
               scan: Union[client.LidarScan,
                           Tuple[Optional[client.LidarScan]]],
               scan_num: Optional[int] = None) -> None:
        self._scan = [scan] if isinstance(scan, client.LidarScan) else scan
        if scan_num is not None:
            self._scan_num = scan_num
        else:
            self._scan_num += 1

    def draw(self, update: bool = True) -> bool:
        """Process and draw the latest state to the screen."""
        with self._lock:
            self._draw()

        if not self._image_size_initialized:
            self.update_image_size(0)
            self._image_size_initialized = True

        if update:
            return self._viz.update()
        else:
            return False

    # i/o and processing, called from client thread
    # usually need to synchronize with key handlers, which run in render thread
    def _draw(self) -> None:

        # update combined Auto Exposure for multiple frames
        if self._ae_enabled:
            if self._cloud_mode == MultiLidarScanViz.CloudMode.RANGE:
                cloud_keys = np.empty(0)
                for idx, ls in enumerate(self._scan):
                    if self._sensor_enabled[idx] and ls is not None:
                        cloud_keys = np.concatenate(
                            (cloud_keys, ls.field(ChanField.RANGE).ravel()))
                self._ae_range(cloud_keys.reshape((-1, 1)), update_state=True)

        palette = self._cloud_palette
        self._cloud_palette = None

        for idx, ls in enumerate(self._scan):

            if ls is not None:

                self._clouds[idx].set_range(ls.field(ChanField.RANGE))

                if self._cloud_mode == MultiLidarScanViz.CloudMode.REFLECTIVITY:
                    key_data = ls.field(ChanField.REFLECTIVITY).astype(
                        np.float32) / 255.0

                    self._clouds[idx].set_key(key_data)
                    self._images[idx].set_image(
                        client.destagger(self._metas[idx], key_data))
                elif self._cloud_mode == MultiLidarScanViz.CloudMode.SIGNAL:
                    key_data = ls.field(ChanField.SIGNAL).astype(np.float32)
                    if self._ae_enabled:
                        self._ae_signal[idx](key_data)
                    else:
                        key_data = key_data / np.max(key_data)

                    self._clouds[idx].set_key(key_data)
                    self._images[idx].set_image(
                        client.destagger(self._metas[idx], key_data))
                elif self._cloud_mode == MultiLidarScanViz.CloudMode.NEAR_IR:
                    key_data = ls.field(ChanField.NEAR_IR).astype(np.float32)
                    self._buc_nearir[idx](key_data)
                    if self._ae_enabled:
                        self._ae_nearir[idx](key_data)
                    else:
                        key_data = key_data / np.max(key_data)

                    self._clouds[idx].set_key(key_data)
                    self._images[idx].set_image(
                        client.destagger(self._metas[idx], key_data))
                elif self._cloud_mode == MultiLidarScanViz.CloudMode.RANGE:
                    key_data = ls.field(ChanField.RANGE).astype(np.float32)
                    if self._ae_enabled:
                        self._ae_range(key_data, update_state=False)
                    else:
                        key_data = key_data / np.max(key_data)

                    self._clouds[idx].set_key(key_data)
                    self._images[idx].set_image(
                        client.destagger(self._metas[idx], key_data))
                elif self._cloud_mode == MultiLidarScanViz.CloudMode.RGB:
                    r = ls.field(ChanField.CUSTOM0)
                    g = ls.field(ChanField.CUSTOM1)
                    b = ls.field(ChanField.CUSTOM2)

                    normalizer = 255

                    # for types other than uint8 for RED, GREEN, BLUE channels
                    # we try to check are there really value bigger than 255
                    if (r.dtype != np.uint8 or g.dtype != np.uint8
                            or b.dtype != np.uint8):
                        max_rgb = np.max((np.max(r), np.max(g), np.max(b)))
                        if max_rgb > 255:
                            normalizer = 65535

                    r = (r / normalizer).clip(0, 1.0).astype(np.float32)
                    g = (g / normalizer).clip(0, 1.0).astype(np.float32)
                    b = (b / normalizer).clip(0, 1.0).astype(np.float32)

                    rgb_data = np.dstack((r, g, b))

                    self._clouds[idx].set_key(rgb_data)
                    self._images[idx].set_image(
                        client.destagger(self._metas[idx], rgb_data))

            else:
                key_zeros = np.zeros(
                    (self._metas[idx].format.pixels_per_column,
                     self._metas[idx].format.columns_per_frame, 3))
                self._clouds[idx].set_key(key_zeros)
                self._images[idx].set_image(key_zeros)

            if palette is not None:
                self._clouds[idx].set_palette(palette)

            if self._cloud_mode == MultiLidarScanViz.CloudMode.REFLECTIVITY:
                self._images[idx].set_palette(calref_palette)
            else:
                self._images[idx].clear_palette()

        self._update_multi_viz_osd()
        self._draw_update_scan_poses()

    def _update_multi_viz_osd(self):
        if self._osd_enabled:
            if self._scan is None:
                # TODO: show something in OSD when there is no scan? Well, we
                # shouldn't be in this state ...
                return

            frame_ts = min([
                client.first_valid_packet_ts(s) for s in self._scan
                if s is not None
            ])

            sensors_str = "  ".join([
                f"{num}{'  ' if not enabled else '-' if scan is None else '*'}"
                for num, (enabled, scan) in enumerate(
                    zip(self._sensor_enabled, self._scan), start=1)
            ])
            source_str = ""
            if self._source_name:
                source_str += "\nsource: " + os.path.basename(
                    self._source_name.rstrip(os.sep)).replace("_", "..")

            if (self._ae_enabled and self._cloud_mode not in [
                    MultiLidarScanViz.CloudMode.REFLECTIVITY,
                    MultiLidarScanViz.CloudMode.RGB
            ]):
                ae_str = '(AE)'
            else:
                ae_str = ''

            # extension point for the OSD text, inserts before the "axes" line
            osd_str_extra = self._osd_text_extra()
            if osd_str_extra:
                osd_str_extra += "\n"

            self._osd.set_text(f"{osd_str_extra}frame ts: {frame_ts}\n"
                               f"sensors:  {sensors_str}\n"
                               f"cloud mode: {self._cloud_mode.name}"
                               f" {ae_str}"
                               f"{source_str if source_str else ''}")
        else:
            self._osd.set_text("")

    def _draw_update_scan_poses(self) -> None:
        """Apply poses from the Scans to the scene"""

        # handle Axis and Camera poses
        if self._scan_poses_enabled:

            # scan with the minimal timestamp determines the
            # center of the system (by it's scan pose)
            min_scan_ts_idx = min(
                [i for i, s in enumerate(self._scan) if s is not None],
                key=lambda i: client.first_valid_packet_ts(self._scan[i]))

            pose = client.first_valid_column_pose(self._scan[min_scan_ts_idx])

            self._viz.camera.set_target(np.linalg.inv(pose))
            self._scan_axis_origin.pose = pose

            # update all sensor axis positions
            for axis, m in zip(self._scan_axis, self._metas):
                axis.pose = pose @ m.extrinsic

        else:
            # without poses camera always points to the origin
            self._viz.camera.set_target(np.eye(4))

            self._scan_axis_origin.pose = np.eye(4)
            for axis, m in zip(self._scan_axis, self._metas):
                axis.pose = m.extrinsic

        # handle Cloud poses
        for idx, ls in enumerate(self._scan):
            if ls is not None:
                if self._scan_poses_enabled:
                    self._clouds[idx].set_column_poses(ls.pose)
                else:
                    self._clouds[idx].set_column_poses(
                        self._column_poses_identity[idx])

    def toggle_scan_poses(self) -> None:
        """Toggle the scan poses use"""
        with self._lock:
            if self._scan_poses_enabled:
                self._scan_poses_enabled = False
                print("MuliScanViz: Key SHIFT-T: Scan Poses: OFF")
            else:
                self._scan_poses_enabled = True
                print("MultiScanViz: Key SHIFT-T: Scan Poses: ON")
        return True

    def update_ring_size(self, amount: int) -> None:
        """Change distance ring size."""
        with self._lock:
            self._ring_size = min(3, max(-2, self._ring_size + amount))
            self._viz.target_display.set_ring_size(self._ring_size)
        return False

    def cicle_ring_line_width(self) -> None:
        """Change rings line width."""
        with self._lock:
            self._ring_line_width = max(1, (self._ring_line_width + 1) % 10)
            self._viz.target_display.set_ring_line_width(self._ring_line_width)
        return False

    def toggle_osd(self, state: Optional[bool] = None) -> None:
        """Show or hide the on-screen display."""
        with self._lock:
            self._osd_enabled = not self._osd_enabled if state is None else state
            print("Toggle OSD to: ", self._osd_enabled)
        self.update_image_size(0)
        return True

    def toggle_scan_axis(self) -> None:
        """Toggle the helper axis of a scan ON/OFF"""
        with self._lock:
            if self._scan_axis_enabled:
                print("MultiLidarScanViz: Key H: Scan Axis Helper - OFF")
                self._scan_axis_enabled = False
                self._scan_axis_origin.disable()
                for axis in self._scan_axis:
                    axis.disable()
            else:
                print("MultiLidarScanViz: Key H: Scan Axis Helper - ON")
                self._scan_axis_enabled = True
                self._scan_axis_origin.enable()
                for axis, enabled in zip(self._scan_axis, self._sensor_enabled):
                    if enabled:
                        axis.enable()
        return False

    def toggle_sensor(self, i: int) -> None:
        """Toggle whether the i'th sensor data is displayed."""
        if i >= len(self._metas):
            return
        with self._lock:
            if self._sensor_enabled[i]:
                self._sensor_enabled[i] = False
                self._viz.remove(self._clouds[i])
                self._viz.remove(self._images[i])
                self._scan_axis[i].disable()
            else:
                self._sensor_enabled[i] = True
                self._viz.add(self._clouds[i])
                self._viz.add(self._images[i])
                if self._scan_axis_enabled:
                    self._scan_axis[i].enable()
        self.update_image_size(0)
        self._update_multi_viz_osd()
        return False

    def toggle_auto_exposure(self) -> None:
        """Toggle the AutoExposure use for 2d images ans clouds"""
        with self._lock:
            if self._ae_enabled:
                print("MultiLidarScanViz: Key SHIFT-A: AutoExposure - OFF")
                self._ae_enabled = False
            else:
                print("MultiLidarScanViz: Key SHIFT-A: AutoExposure - ON")
                self._ae_enabled = True
        return True

    def update_point_size(self, amount: int) -> None:
        """Change the point size of all clouds."""
        with self._lock:
            self._cloud_pt_size = min(10.0,
                                      max(1.0, self._cloud_pt_size + amount))
            for cloud in self._clouds:
                cloud.set_point_size(self._cloud_pt_size)
        return False

    @staticmethod
    def _next_cloud_mode(
            mode: 'MultiLidarScanViz.CloudMode') -> 'MultiLidarScanViz.CloudMode':
        return MultiLidarScanViz.CloudMode(
            (mode.value + 1) % len(MultiLidarScanViz.CloudMode.__members__))

    def _cloud_mode_available(self, mode: 'MultiLidarScanViz.CloudMode') -> bool:
        """Checks whether data is present for the correspoding Cloud mode in all scans"""
        if self._scan is None:
            return True
        if mode in self._mode_to_channels:
            mode_chans = self._mode_to_channels[mode]
            for ls in self._scan:
                if ls is not None and not all(
                    [ch in ls.fields for ch in mode_chans]):
                    return False
            return True
        return False

    def _next_available_cloud_mode(
        self, mode: 'MultiLidarScanViz.CloudMode'
    ) -> Optional['MultiLidarScanViz.CloudMode']:
        """Switch cloud mode to the next available that can be drawn"""
        next_mode = self._next_cloud_mode(mode)
        cnt = 0
        while (cnt < len(MultiLidarScanViz.CloudMode.__members__)
               and not self._cloud_mode_available(next_mode)):
            next_mode = self._next_cloud_mode(next_mode)
            cnt += 1
        if cnt < len(MultiLidarScanViz.CloudMode.__members__):
            return next_mode
        else:
            return None

    def update_cloud_mode(self,
                          mode: 'Optional[MultiLidarScanViz.CloudMode]' = None
                          ) -> None:
        with self._lock:
            # cycle between cloud mode enum values
            if mode is None:
                next_mode = self._next_available_cloud_mode(self._cloud_mode)
                if next_mode is not None:
                    self._cloud_mode = next_mode
                else:
                    print("ERROR: no cloud mode has data in frames")
            else:
                if self._cloud_mode_available(mode):
                    self._cloud_mode = mode

            if self._cloud_mode == MultiLidarScanViz.CloudMode.REFLECTIVITY:
                self._cloud_palette = calref_palette
            else:
                self._cloud_palette = spezia_palette

            print("Cloud Mode: ", self._cloud_mode)
        return True

    def update_images_layout(
            self,
            layout: 'Optional[MultiLidarScanViz.ImagesLayout]' = None) -> None:
        with self._lock:
            # cycle between images layout enum values
            if layout is None:
                self._images_layout = MultiLidarScanViz.ImagesLayout(
                    (self._images_layout.value + 1) %
                    len(MultiLidarScanViz.ImagesLayout.__members__))
            else:
                self._images_layout = layout
            print("Images Layout: ", self._images_layout)
        self.update_image_size(0)
        return False
