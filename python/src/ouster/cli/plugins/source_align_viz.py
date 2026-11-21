"""Interactive extrinsics editor for ouster-cli (``align_viz``).

Displays the point cloud for each sensor in the first frame set of a
multi-sensor dataset and lets the user interactively adjust each sensor's
``sensor_to_body`` extrinsic with the mouse and keyboard, then write the result
to an extrinsics json file.
"""
from typing import List, Optional, Tuple

import click
import numpy as np

from ouster.sdk.core import FrameSet, CameraInfo
from ouster.cli.plugins.source import (source,  # type: ignore
                                       write_extrinsics_json,
                                       _ALIGN_FRAME_FILL_WINDOW)
from ouster.cli.plugins.source_util import (CoupledTee,
                                            SourceCommandContext,
                                            SourceCommandType,
                                            source_multicommand)


# Interaction scales
_TRANSLATE_M_PER_PIXEL = 0.01
_ROTATE_RAD_PER_PIXEL = 0.005


class EditMode:
    NONE = "orbit view"
    XY_TRANSLATE = "translate XY"
    Z_TRANSLATE = "translate Z"
    ROT_X = "rotate X"
    ROT_Y = "rotate Y"
    ROT_Z = "rotate Z"


def _translation(dx: float, dy: float, dz: float) -> np.ndarray:
    m = np.eye(4, dtype=np.float64)
    m[0, 3] = dx
    m[1, 3] = dy
    m[2, 3] = dz
    return m


def _rotation_about_axis(axis: np.ndarray, angle: float) -> np.ndarray:
    """Return a 4x4 homogeneous rotation of ``angle`` radians about ``axis``."""
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.eye(4, dtype=np.float64)
    x, y, z = (axis / norm)
    c = np.cos(angle)
    s = np.sin(angle)
    cc = 1.0 - c
    m = np.eye(4, dtype=np.float64)
    m[:3, :3] = np.array([
        [c + x * x * cc, x * y * cc - z * s, x * z * cc + y * s],
        [y * x * cc + z * s, c + y * y * cc, y * z * cc - x * s],
        [z * x * cc - y * s, z * y * cc + x * s, c + z * z * cc],
    ], dtype=np.float64)
    return m


class ExtrinsicsEditController:
    """Wires mouse/keyboard input to per-sensor extrinsic editing on a viz."""

    def __init__(self, point_viz, lidar_frame_viz, sensor_info: List,
                 viz_module, output: Optional[str], overwrite: bool,
                 frames_iter, selected: FrameSet) -> None:
        self._viz = point_viz
        self._lfv = lidar_frame_viz
        self._sensor_info = sensor_info
        self._vm = viz_module
        self._output = output
        self._overwrite = overwrite

        self._num_sensors = len(sensor_info)
        self._frames_iter = frames_iter
        self._selected = selected
        self._frame_num = 0

        self._current = 0
        self._mode = EditMode.NONE
        self._dragging = False

        # undo history: each entry snapshots (sensor index, delta) from just
        # before a contiguous adjustment (one mouse drag). _drag_recorded guards
        # against pushing more than one snapshot per drag.
        self._undo_stack: List[Tuple[int, np.ndarray]] = []
        self._drag_recorded = False

        # Map each sensor_info index to its viz model unit. The model keeps
        # lidar sensors and cameras in separate lists (_sensors / _cameras),
        # each preserving per-type order, so walk them in parallel with the
        # combined sensor_info order.
        model = lidar_frame_viz._model
        lidar_iter = iter(model._sensors)
        camera_iter = iter(model._cameras)
        # Scene axis helpers live in a single _stream_axis list ordered like
        # _stream_models() (lidar sensors first, then cameras). Slice it back
        # into per-type iterators so they line up with model._sensors /
        # model._cameras walked above.
        n_lidar = len(model._sensors)
        lidar_axis_iter = iter(lidar_frame_viz._stream_axis[:n_lidar])
        camera_axis_iter = iter(lidar_frame_viz._stream_axis[n_lidar:])

        # base_ext is the sensor_to_body transform already baked into each
        # displayed cloud (into the lidar XYZLut, and into the camera cloud's
        # world points). We track a world-frame delta applied via set_pose,
        # giving an effective extrinsic of delta @ base_ext (no inversion, so
        # singular/unset extrinsics are handled gracefully).
        self._units: List = []
        self._axes: List = []
        self._base_ext: List[np.ndarray] = []
        for i, si in enumerate(sensor_info):
            if isinstance(si, CameraInfo):
                self._units.append(next(camera_iter))
                self._axes.append(next(camera_axis_iter, None))
            else:
                self._units.append(next(lidar_iter))
                self._axes.append(next(lidar_axis_iter, None))
            base = np.asarray(si.sensor_to_body, dtype=np.float64).copy()
            if base.shape != (4, 4) or abs(np.linalg.det(base)) < 1e-12:
                click.secho(
                    f"Warning: sensor {i + 1} has a singular/unset extrinsic; "
                    "using identity as its base.", fg='yellow')
                base = np.eye(4, dtype=np.float64)
            self._base_ext.append(base)
        self._delta = [np.eye(4, dtype=np.float64) for _ in sensor_info]

        # the axis helper base pose (with an identity delta) is recomputed by
        # the model on every draw; snapshot it so the interactive delta can be
        # composed on top and re-applied after draws
        self._base_axis_pose: List[Optional[np.ndarray]] = \
            [None] * self._num_sensors
        self._capture_base_axis_poses()

        self._mod_shift = int(viz_module.EventModifierKeys.MOD_SHIFT)
        self._mod_ctrl = int(viz_module.EventModifierKeys.MOD_CONTROL)

        # LidarFrameViz redraws itself after many events (config changes and key
        # bindings like OSD 'O'), and each draw() recomputes the axis helper
        # poses to the sensor base (identity delta). Wrap draw() so our
        # per-sensor delta is recaptured and re-applied to the clouds and axis
        # helpers after every such redraw, keeping them in sync.
        self._orig_lfv_draw = self._lfv.draw
        self._lfv.draw = self._draw_and_reapply  # type: ignore[method-assign]

        # push handlers after LidarFrameViz init so they take input priority
        self._viz.push_key_handler(self._on_key)
        self._viz.push_mouse_button_handler(self._on_mouse_button)
        self._viz.push_mouse_pos_handler(self._on_mouse_pos)

        self._notify()

    def _draw_and_reapply(self, *args, **kwargs):
        result = self._orig_lfv_draw(*args, **kwargs)
        self._capture_base_axis_poses()
        self._reapply_all()
        # draw() renders (self._viz.update()) before returning, i.e. before our
        # re-apply above, so force another render to show the corrected poses
        self._viz.update()
        return result

    # --- helpers ------------------------------------------------------------
    def _sensor_clouds(self, index: int):
        return self._units[index]._clouds

    def _capture_base_axis_poses(self) -> None:
        """Snapshot each sensor's axis pose as computed by the model (delta=I)."""
        for i, axis in enumerate(self._axes):
            if axis is not None:
                self._base_axis_pose[i] = np.asarray(
                    axis.pose, dtype=np.float64).copy()

    def _apply_sensor(self, index: int) -> None:
        pose = np.ascontiguousarray(self._delta[index], dtype=np.float64)
        for cloud in self._sensor_clouds(index):
            cloud.set_pose(pose)
        # move the sensor's axis helper with the cloud
        axis = self._axes[index]
        base_axis = self._base_axis_pose[index]
        if axis is not None and base_axis is not None:
            axis.pose = self._delta[index] @ base_axis

    def _apply(self) -> None:
        self._apply_sensor(self._current)
        self._viz.update()

    def _reapply_all(self) -> None:
        for i in range(self._num_sensors):
            self._apply_sensor(i)

    def _notify(self) -> None:
        try:
            self._viz.set_notification(
                f"Sensor {self._current + 1}/{self._num_sensors}  |  "
                f"mode: {self._mode}")
        except Exception:
            pass

    def _toggle_mode(self, mode: str) -> None:
        # toggling the active mode returns to the default (orbit) behavior
        self._mode = EditMode.NONE if self._mode == mode else mode
        self._notify()

    def _begin_edit(self, index: int) -> None:
        """Record a sensor's delta once at the start of a contiguous drag."""
        if not self._drag_recorded:
            self._undo_stack.append((index, self._delta[index].copy()))
            self._drag_recorded = True

    def _undo(self) -> None:
        """Revert the most recent contiguous adjustment (one drag)."""
        if not self._undo_stack:
            self._viz.set_notification("Nothing to undo.")
            return
        index, delta = self._undo_stack.pop()
        self._delta[index] = delta
        self._apply_sensor(index)
        self._viz.update()
        self._viz.set_notification(f"Undid last adjustment (sensor {index + 1}).")

    def _advance(self) -> None:
        """Advance the source by one frame set and refresh the display."""
        try:
            nxt = next(self._frames_iter)
        except StopIteration:
            self._viz.set_notification("End of source reached.")
            return
        # keep the previous frame for any sensor missing in the new set
        merged = FrameSet([None] * self._num_sensors)
        for i in range(self._num_sensors):
            frame = nxt[i] if (nxt and i < len(nxt)) else None
            merged[i] = frame if frame is not None else self._selected[i]
        self._selected = merged
        self._frame_num += 1
        self._lfv.update(merged, frame_num=self._frame_num)
        # the wrapped draw() recaptures the base axis poses, re-applies the
        # per-sensor delta to both clouds and axes, and re-renders
        self._lfv.draw()
        self._viz.set_notification(f"Frame set {self._frame_num}")

    def _save(self) -> None:
        if self._output is None:
            self._viz.set_notification(
                "No output path set; use -o/--output to enable saving.")
            return
        # copy live edits back into sensor_info before serializing
        for i, si in enumerate(self._sensor_info):
            si.sensor_to_body = self._delta[i] @ self._base_ext[i]
            si.clear_cache()
        try:
            write_extrinsics_json(self._sensor_info, self._output,
                                  self._overwrite)
            self._viz.set_notification(f"Saved extrinsics to {self._output}")
        except FileExistsError:
            self._viz.set_notification(
                f"{self._output} exists; pass --overwrite to replace it.")
        except Exception as e:
            self._viz.set_notification(f"Failed to save extrinsics: {e}")

    # --- transforms ---------------------------------------------------------
    def _translate(self, dx_m: float, dy_m: float, dz_m: float) -> None:
        i = self._current
        self._begin_edit(i)
        self._delta[i] = _translation(dx_m, dy_m, dz_m) @ self._delta[i]
        self._apply()

    def _rotate_local_axis(self, axis_col: int, angle: float) -> None:
        i = self._current
        self._begin_edit(i)
        # current world pose of the sensor = delta @ base_ext; rotate about the
        # sensor's own axis passing through its current position
        world = self._delta[i] @ self._base_ext[i]
        pivot = world[:3, 3].copy()
        axis_world = world[:3, axis_col].copy()
        step = (_translation(*pivot) @ _rotation_about_axis(axis_world, angle)
                @ _translation(*(-pivot)))
        self._delta[i] = step @ self._delta[i]
        self._apply()

    # --- input handlers -----------------------------------------------------
    def _on_key(self, ctx, key: int, mods: int) -> bool:
        # number keys select the current sensor; Ctrl+number toggles that
        # sensor's visibility (like LidarScanViz)
        if ord('1') <= key <= ord('9'):
            index = key - ord('1')
            if index >= len(self._sensor_info):
                return False
            if mods & self._mod_ctrl:
                self._lfv.toggle_sensor(index)
                enabled = getattr(self._units[index], '_enabled', True)
                self._lfv.draw()
                self._viz.set_notification(
                    f"Sensor {index + 1} "
                    f"{'shown' if enabled else 'hidden'}")
                return False
            self._current = index
            self._notify()
            return False
        if key == ord('.') and mods == 0:
            self._advance()
            return False
        # disable camera follow: swallow 'U' (and shift+'U') so LidarFrameViz
        # never cycles the camera mode in the alignment tool
        if key == ord('U'):
            return False
        if key == ord('T') and mods == 0:
            self._toggle_mode(EditMode.XY_TRANSLATE)
            return False
        if key == ord('Z') and (mods & self._mod_ctrl):
            self._undo()
            return False
        if key == ord('Z') and mods == 0:
            self._toggle_mode(EditMode.Z_TRANSLATE)
            return False
        if key == ord('Z') and (mods & self._mod_shift):
            self._toggle_mode(EditMode.ROT_Z)
            return False
        if key == ord('X') and (mods & self._mod_shift):
            self._toggle_mode(EditMode.ROT_X)
            return False
        if key == ord('Y') and (mods & self._mod_shift):
            self._toggle_mode(EditMode.ROT_Y)
            return False
        if key == ord('S') and (mods & self._mod_ctrl):
            self._save()
            return False
        return True

    def _on_mouse_button(self, ctx, button, event, mods) -> bool:
        if button == self._vm.MouseButton.MOUSE_BUTTON_LEFT:
            if event == self._vm.MouseButtonEvent.MOUSE_BUTTON_PRESSED:
                # only capture the drag when an edit mode is active; otherwise
                # let the default camera controls orbit the view
                if self._mode == EditMode.NONE:
                    return True
                self._dragging = True
                # start of a new contiguous adjustment; the snapshot is taken
                # lazily on the first actual transform so empty drags (press
                # with no movement) do not create undo entries
                self._drag_recorded = False
                return False
            if event == self._vm.MouseButtonEvent.MOUSE_BUTTON_RELEASED:
                if not self._dragging:
                    return True
                self._dragging = False
                return False
        return True

    def _on_mouse_pos(self, ctx, x: float, y: float) -> bool:
        if not self._dragging or self._mode == EditMode.NONE:
            return True
        dx = float(x - ctx.mouse_x)
        dy = float(y - ctx.mouse_y)
        if self._mode == EditMode.XY_TRANSLATE:
            self._translate(dx * _TRANSLATE_M_PER_PIXEL,
                            -dy * _TRANSLATE_M_PER_PIXEL, 0.0)
        elif self._mode == EditMode.Z_TRANSLATE:
            self._translate(0.0, 0.0, -dy * _TRANSLATE_M_PER_PIXEL)
        elif self._mode == EditMode.ROT_X:
            self._rotate_local_axis(0, dx * _ROTATE_RAD_PER_PIXEL)
        elif self._mode == EditMode.ROT_Y:
            self._rotate_local_axis(1, dx * _ROTATE_RAD_PER_PIXEL)
        elif self._mode == EditMode.ROT_Z:
            self._rotate_local_axis(2, dx * _ROTATE_RAD_PER_PIXEL)
        return False


def _collect_first_frame_set(frames_iter, num_sensors: int) -> FrameSet:
    """Assemble the first available frame per sensor from the first frame sets.

    A frame is accepted as soon as it is present (non-None), matching the
    behavior of ``viz`` on the first frame. This intentionally does not require
    a lidar RANGE point cloud, so non-lidar sensors (e.g. cameras with RANGE)
    are displayed as well. The fill window only exists to gather sensors that
    first appear in a later frame set (time skew).
    """
    selected = FrameSet([None] * num_sensors)
    num_frames = 0
    for frames in frames_iter:
        if not frames:
            continue
        num_frames += 1
        for sensor_idx, frame in enumerate(frames):
            if sensor_idx >= num_sensors:
                continue
            if frame is not None and selected[sensor_idx] is None:
                selected[sensor_idx] = frame
        if num_frames >= _ALIGN_FRAME_FILL_WINDOW:
            break
        if all(selected[i] is not None for i in range(num_sensors)):
            break
    return selected


@click.command(name="align_viz", hidden=True)
@click.option('-o', '--output', type=click.Path(), default=None,
              help="Path to write the edited extrinsics json file (Ctrl+S).")
@click.option('--overwrite', is_flag=True, default=False,
              help="If true, overwrite an existing output file.")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_align_viz(ctx: SourceCommandContext, output: Optional[str],
                     overwrite: bool) -> None:
    """BETA feature, subject to change or removal.

    Interactively align multi-sensor extrinsics in a 3D view.

    Number keys select the current sensor; Ctrl+number shows/hides that sensor.
    Left-click and drag orbits the view.
    Press 't' to toggle XY translation, 'z' to toggle Z translation, and
    'shift+z' / 'shift+x' / 'shift+y' to toggle rotation about the sensor's own
    z/x/y axis; while a mode is active, left-drag applies that transform to the
    selected sensor. Press '.' to advance the source by one frame set. Ctrl+Z
    undoes the most recent adjustment. Ctrl+S saves the extrinsics to the file
    given by -o/--output.
    """
    try:
        from ouster.sdk import viz as viz_module
        from ouster.sdk.viz import LidarFrameViz
    except ImportError as e:
        raise click.ClickException(str(e))

    source_obj = ctx.frame_set_source
    assert source_obj is not None
    assert ctx.frame_set_iter is not None

    sensor_info = list(source_obj.sensor_info)
    num_sensors = len(sensor_info)

    ctx.frame_set_iter, tees = CoupledTee.tee(ctx.frame_set_iter,  # type: ignore
                                              terminate=ctx.terminate_evt,
                                              loop=False)
    frames_tee = tees[0]

    def viz_thread_fn():
        selected = _collect_first_frame_set(frames_tee, num_sensors)
        if all(selected[i] is None for i in range(num_sensors)):
            click.secho("Warning: align_viz could not read any usable frame; "
                        "skipping.", fg='yellow')
            assert ctx.terminate_evt is not None
            ctx.terminate_evt.set()
            return

        title = "Ouster Align Viz" + (f": {ctx.source_uri}" if ctx.source_uri else "")
        point_viz = viz_module.PointViz(title)
        lfv = LidarFrameViz(sensor_info, point_viz)
        lfv.update(selected, frame_num=0)
        lfv.draw()

        # keep a reference so the input handlers stay alive during run()
        controller = ExtrinsicsEditController(  # noqa: F841
            point_viz, lfv, sensor_info, viz_module, output, overwrite,
            frames_tee, selected)

        print("align_viz: number keys select a sensor; Ctrl+number shows/hides "
              "a sensor; left-drag orbits the view; 't' toggles XY-translate; "
              "'z' toggles Z-translate; shift+z/x/y toggle rotation; '.' "
              "advances one frame set; Ctrl+Z undoes the last adjustment; "
              "Ctrl+S saves.")
        point_viz.run()
        assert ctx.terminate_evt is not None
        ctx.terminate_evt.set()

    if ctx.main_thread_fn is not None:
        raise RuntimeError("A main-thread required function has already been set.")
    ctx.main_thread_fn = viz_thread_fn  # type: ignore


source.commands['ANY']['align_viz'] = source_align_viz
