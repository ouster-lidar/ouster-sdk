from __future__ import annotations

import os
import threading
from functools import partial
from types import ModuleType
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

import ouster.sdk._bindings.mapping as mapping
from ouster.sdk import open_source, viz as _viz
from ouster.sdk.core import get_rot_matrix_to_align_to_gravity
from ouster.sdk._bindings.viz import (
    Cloud,
    EventModifierKeys,
    Label,
    Lines,
    MouseButton,
    MouseButtonEvent,
    PointViz,
    WindowCtx,
)


viz: ModuleType = _viz


def _as_f_pose(matrix: np.ndarray) -> np.ndarray:
    pose = np.asarray(matrix, dtype=np.float64)
    if pose.shape == (4, 4):
        return np.asfortranarray(pose)
    if pose.ndim == 1 and pose.size == 16:
        return np.asfortranarray(pose.reshape((4, 4), order="F"))
    if pose.ndim == 2 and pose.size == 16:
        return np.asfortranarray(pose.reshape((4, 4), order="F"))
    raise ValueError("pose must be 4x4")


def _transform_xyz(xyz: np.ndarray, pose: np.ndarray) -> np.ndarray:
    points = np.asarray(xyz, dtype=np.float64)
    transform = _as_f_pose(pose)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("xyz must have shape (N, 3)")
    if points.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float32)
    rotated = points @ transform[:3, :3].T
    translated = rotated + transform[:3, 3]
    return translated.astype(np.float32, copy=False)


class PoseOptimizerViz:
    def __init__(
        self,
        pose_optimizer: Any,
        output_path: str,
        input_source_uri: Optional[str] = None,
        align_with_absolute_constraints: Optional[bool] = None,
        loop_cell_size_m: Optional[float] = None,
        loop_min_distance_m: float = 50.0,
        loop_icp_threshold: float = 0.0,
    ) -> None:
        self._viz_module: ModuleType = viz

        self.po = pose_optimizer
        if align_with_absolute_constraints is None:
            align_with_absolute_constraints = True
        self._align_on_start = bool(align_with_absolute_constraints)
        try:
            loop_cell_size_val = float(loop_cell_size_m) if loop_cell_size_m is not None else None
        except Exception:
            loop_cell_size_val = None
        self.loop_cell_size_m: Optional[float] = None
        if loop_cell_size_val is not None and loop_cell_size_val > 0:
            self.loop_cell_size_m = loop_cell_size_val
        try:
            loop_min_distance_val = float(loop_min_distance_m)
        except Exception:
            loop_min_distance_val = 50.0
        if loop_min_distance_val <= 0.0:
            loop_min_distance_val = 50.0
        self.loop_min_distance_m = loop_min_distance_val
        try:
            loop_icp_threshold_val = float(loop_icp_threshold)
        except Exception:
            loop_icp_threshold_val = 0.0
        if loop_icp_threshold_val < 0.0:
            loop_icp_threshold_val = 0.0
        if loop_icp_threshold_val > 1.0:
            loop_icp_threshold_val = 1.0
        self.loop_icp_threshold = loop_icp_threshold_val

        self.output_path = os.path.abspath(output_path)
        self.input_source_uri = input_source_uri or ""
        base, ext = os.path.splitext(self.output_path)
        self.output_osf_filename = base + (ext or ".osf")

        try:
            self.key_frame_distance = float(self.po.get_key_frame_distance())
        except Exception:
            self.key_frame_distance = float('nan')

        self.sampling_mode = mapping.SamplingMode.KEY_FRAMES
        self.viz: PointViz = self._viz_module.PointViz("Pose Optimizer", maximized=True)

        # Optimized trajectory cloud/lines
        self.opt_cloud: Cloud = self._viz_module.Cloud(1)
        self.opt_cloud.set_point_size(4.0)
        self.opt_cloud.set_key(
            np.array([[1.0, 0.5, 0.0, 1.0]], dtype=np.float32)
        )
        self.viz.add(self.opt_cloud)
        self.opt_line: Lines = self._viz_module.Lines(
            np.eye(4, dtype=np.float64),
            (1.0, 0.5, 0.0, 1.0),
        )
        self.viz.add(self.opt_line)

        # Original trajectory background
        try:
            poses0 = self.po.get_poses(self.sampling_mode)
            xyz0 = np.asarray([np.asarray(T, dtype=np.float64)[:3, 3]
                               for T in poses0],
                              dtype=np.float32)
        except Exception:
            xyz0 = np.zeros((0, 3), dtype=np.float32)
        self._raw_xyz = xyz0
        try:
            base_ts = self.po.get_timestamps(mapping.SamplingMode.KEY_FRAMES)
            self._keyframe_ts_base = [int(ts) for ts in base_ts]
        except Exception:
            self._keyframe_ts_base = []
        self.raw_cloud: Cloud = self._viz_module.Cloud(max(1, xyz0.shape[0]))
        self.raw_cloud.set_point_size(3.0)
        if xyz0.size:
            self.raw_cloud.set_xyz(xyz0)
        rgba_bg = np.tile(np.array([1.0, 1.0, 1.0, 0.6], dtype=np.float32),
                          (max(1, xyz0.shape[0]), 1))
        self.raw_cloud.set_key(rgba_bg)
        self.viz.add(self.raw_cloud)
        self.raw_line: Lines = self._viz_module.Lines(
            np.eye(4, dtype=np.float64),
            (0.8, 0.8, 0.8, 0.6),
        )
        if xyz0.shape[0] > 1:
            self.raw_line.set_points(self._make_line_segments(xyz0))
        self.viz.add(self.raw_line)

        # Constraint visuals
        self.lines_ptp: Lines = self._viz_module.Lines(
            np.eye(4, dtype=np.float64),
            (0.0, 1.0, 0.0, 1.0),
        )
        self.lines_abspt: Lines = self._viz_module.Lines(
            np.eye(4, dtype=np.float64),
            (0.0, 1.0, 0.0, 1.0),
        )
        self.lines_abspose: Lines = self._viz_module.Lines(
            np.eye(4, dtype=np.float64),
            (0.0, 1.0, 0.0, 1.0),
        )
        self.lines_selected: Lines = self._viz_module.Lines(
            np.eye(4, dtype=np.float64),
            (1.0, 1.0, 0.0, 1.0),
        )
        self.viz.add(self.lines_ptp)
        self.viz.add(self.lines_abspt)
        self.viz.add(self.lines_abspose)
        self.viz.add(self.lines_selected)

        # Orientation indicators
        # RGB colors represent XYZ Euler angles: Red=X(Roll), Green=Y(Pitch), Blue=Z(Yaw)
        self.node_axis_lines: List[Lines] = []
        for color in ((1.0, 0.0, 0.0, 1.0),   # Red for X-axis (Roll)
                      (0.0, 1.0, 0.0, 1.0),   # Green for Y-axis (Pitch)
                      (0.0, 0.0, 1.0, 1.0)):  # Blue for Z-axis (Yaw)
            axis_lines = self._viz_module.Lines(np.eye(4, dtype=np.float64), color)
            self.viz.add(axis_lines)
            self.node_axis_lines.append(axis_lines)
        self._orientation_length = 1.0

        self.abs_pose_axis_lines: List[Lines] = []
        for color in ((1.0, 0.0, 0.0, 0.6),   # Red for X-axis (Roll)
                      (0.0, 1.0, 0.0, 0.6),   # Green for Y-axis (Pitch)
                      (0.0, 0.0, 1.0, 0.6)):  # Blue for Z-axis (Yaw)
            axis_lines = self._viz_module.Lines(np.eye(4, dtype=np.float64), color)
            axis_lines.set_points(np.zeros((0, 3), dtype=np.float32))
            self.viz.add(axis_lines)
            self.abs_pose_axis_lines.append(axis_lines)

        self.show_abs_pose_orientations = True
        self._abs_pose_axes_buffer: List[np.ndarray] = []

        # Absolute target markers
        self.targets_cloud: Cloud = self._viz_module.Cloud(1)
        self.targets_cloud.set_point_size(6.0)
        self.viz.add(self.targets_cloud)

        self.node_clouds: Dict[int, Cloud] = {}
        self.marker_clouds: Dict[Tuple[int, str], Cloud] = {}
        self.constraint_labels: Dict[int, Label] = {}
        self.constraint_pick_clouds: Dict[int, "Cloud"] = {}
        self.selected_constraint_ids: set[int] = set()
        self._constraint_segments: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
        self._pick_hits: set[int] = set()
        self._pick_samples_per_constraint = 9
        self._left_click_start: Optional[Tuple[float, float]] = None
        self._left_click_moved = False
        self._left_click_threshold_px = 9.0

        self.node_rgba = np.array([1.0, 0.5, 0.0, 0.25], dtype=np.float32)
        self.label_rgba = (1.0, 1.0, 1.0, 0.6)
        self.label_selected_rgba = (1.0, 1.0, 0.2, 1.0)
        self.pose_marker_red = np.array([1.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.pose_marker_blue = np.array([0.0, 0.3, 1.0, 1.0], dtype=np.float32)
        self.cloud_red = np.array([0.95, 0.2, 0.2, 0.35], dtype=np.float32)
        self.cloud_blue = np.array([0.2, 0.55, 1.0, 0.35], dtype=np.float32)
        self.cloud_selected_a = np.array([1.0, 0.95, 0.15, 0.75],
                                         dtype=np.float32)
        self.cloud_selected_b = np.array([0.15, 1.0, 0.9, 0.75],
                                         dtype=np.float32)
        self.sampled_cloud_rgba = np.array([0.0, 0.85, 1.0, 0.55],
                                           dtype=np.float32)

        self.sampled_clouds: List[Tuple[Cloud, int]] = []
        self.sampled_cloud_visible = False

        # Get sampled nodes once at initialization
        self.sampled_nodes: List[Any] = []
        try:
            all_nodes = self.po.get_sampled_nodes(100)
            # Skip the first frame, start with second
            self.sampled_nodes = all_nodes[1:] if len(all_nodes) > 1 else []
        except Exception:
            self.sampled_nodes = []

        self.solver_step_callback = self.step_callback
        self.po.set_solver_step_callback(self.solver_step_callback)
        self._setup_controls()
        self._setup_mouse_pan()

        help_lines = [
            f"Key Frame Distance: {self.key_frame_distance:.2f} m",
            "Ctrl+s: Save the result",
            "n: solve 1 step |  Shift+n: solve 10 steps",
            "Shift+Ctrl+n: solve until convergence (PoseOptimizer iterations)",
            "g: Align viz with gravity",
            "h: Toggle sampled frame clouds",
            "r: Toggle raw trajectory ON/OFF",
            "p: Increase point cloud size | Shift+p: Decrease point cloud size",
            "t: Toggle constraint labels ON/OFF",
            "c: Toggle pose-to-pose cloud highlights ON/OFF",
            "o: Toggle node direction arrows (RGB=XYZ=Roll/Pitch/Yaw)",
            "x: Toggle absolute pose axes (RGB=XYZ=Roll/Pitch/Yaw)",
            "l: Auto-detect loop closures on current trajectory",
            "u: Remove selected constraints",
            "Ctrl+a: Select all constraints",
            "Left click: Select constraint line/node (Shift=add, Ctrl=toggle)",
        ]
        self._help_lines = help_lines
        self._help_label: Optional[Label] = None
        self._remove_button_label: Optional[Label] = None
        self._remove_button_anchor = (0.99, 0.02)
        self._remove_button_size = (0.30, 0.045)
        self._status_text = ""
        try:
            help_label = self._viz_module.Label("", 0.0, 0.0, align_top=True)
            help_label.set_scale(1.2)
            self.viz.add(help_label)
            self._help_label = help_label
            self._update_help_label()
        except Exception:
            self._help_label = None
        try:
            remove_btn = self._viz_module.Label(
                "", self._remove_button_anchor[0], self._remove_button_anchor[1],
                align_right=True, align_top=True
            )
            remove_btn.set_scale(1.2)
            self.viz.add(remove_btn)
            self._remove_button_label = remove_btn
        except Exception:
            self._remove_button_label = None

        self.show_constraint_timestamps = True
        self.show_constraint_labels = True
        self.show_pose_to_pose_clouds = True
        self.show_node_orientations = False
        self._raw_trajectory_visible = True
        self._raw_alignment_applied = False
        self._camera_fitted = False
        self._gravity_view_pose = np.eye(4, dtype=np.float64)
        self._gravity_view_pose_cache: Optional[np.ndarray] = None
        self._gravity_view_pose_cache_valid: Optional[bool] = None

        self._update_remove_button_label()
        self._update_all()
        self.viz.update()

    # --- control setup -------------------------------------------------
    def _setup_controls(self) -> None:
        KEY_N = ord('N')
        KEY_S = ord('S')
        KEY_T = ord('T')
        KEY_O = ord('O')
        KEY_G = ord('G')
        KEY_H = ord('H')
        KEY_C = ord('C')
        KEY_R = ord('R')
        KEY_X = ord('X')
        KEY_L = ord('L')
        KEY_A = ord('A')
        KEY_U = ord('U')
        MOD_SHIFT = int(self._viz_module.EventModifierKeys.MOD_SHIFT)
        MOD_CTRL = int(self._viz_module.EventModifierKeys.MOD_CONTROL)
        self._lock = threading.Lock()
        lock = self._lock

        self._solve_running = False
        self._align_done = False
        self._solving_active = False
        self._solving_total_steps = 0
        self._solving_done_steps = 0
        try:
            self._total_solve_steps = int(self.po.get_total_iterations())
        except Exception:
            self._total_solve_steps = 0
        self._running_until_converged = False
        self._pending_auto_loop = False
        self._prev_cost: Optional[float] = None
        self._convergence_tolerance = 1e-8
        self._cloud_pt_size = 1.0

        def _apply_point_size() -> None:
            for cloud in self.node_clouds.values():
                cloud.set_point_size(self._cloud_pt_size)
            for cloud, _ in self.sampled_clouds:
                cloud.set_point_size(self._cloud_pt_size)
            self.viz.update()

        def _set_status(text: str) -> None:
            self._status_text = text
            try:
                self._update_help_label()
                self.viz.update()
            except Exception:
                pass

        def _toggle_sampled_clouds() -> None:
            if self.sampled_cloud_visible:
                for cloud, _ in self.sampled_clouds:
                    self.viz.remove(cloud)
                self.sampled_clouds.clear()
                self.sampled_cloud_visible = False
                self.viz.update()
                return

            # Use pre-fetched sampled nodes but get current poses
            if not self.sampled_nodes:
                self.viz.update()
                return

            sampled: List[Tuple[Any, int]] = []
            for node in self.sampled_nodes:
                try:
                    ts = int(node.ts)
                except Exception:
                    continue
                pts = np.ascontiguousarray(
                    node.downsampled_pts, dtype=np.float32)
                if pts.size == 0:
                    continue
                try:
                    # Get current optimized pose, not the original one
                    current_node = self.po.get_node(ts)
                    if current_node:
                        pose = _as_f_pose(current_node.get_pose())
                    else:
                        pose = _as_f_pose(node.get_pose())
                except Exception:
                    continue

                cloud = self._viz_module.Cloud(pts.shape[0])
                cloud.set_xyz(pts)
                cloud.set_point_size(self._cloud_pt_size)
                cloud.set_key(np.tile(self.sampled_cloud_rgba, (pts.shape[0], 1)))
                cloud.set_pose(pose)
                self.viz.add(cloud)
                sampled.append((cloud, ts))

            if not sampled:
                self.viz.update()
                return

            self.sampled_clouds = sampled
            self.sampled_cloud_visible = True
            self.viz.update()

        def run_steps(step_count: Optional[int] = None, *, until_converged: bool = False):
            with lock:
                if self._solve_running:
                    return
                self._solve_running = True
            self._running_until_converged = until_converged
            last_cost_value: Optional[float] = None
            diff_val: Optional[float] = None
            converged_successfully = False
            prev_cost_local = self._prev_cost
            try:
                if not self._align_done:
                    if self._align_on_start:
                        try:
                            constraints = self.po.get_constraints()
                            abs_constraints = [
                                c for c in constraints
                                if (hasattr(c, 'pose') and hasattr(c, 'timestamp'))
                                or (hasattr(c, 'absolute_position')
                                    and hasattr(c, 'timestamp'))
                            ]
                            if abs_constraints:
                                alignment_pose = self.po.initialize_trajectory_alignment()
                                self._apply_raw_trajectory_alignment(alignment_pose)
                        except Exception:
                            pass
                    self._align_done = True

                self._solving_active = True
                self._solving_done_steps = 0

                before_total = None
                try:
                    before_total = int(self.po.get_total_iterations())
                except Exception:
                    before_total = None

                if until_converged:
                    self._solving_total_steps = -1
                    _set_status("Solving until convergence (PoseOptimizer limits)")
                    last_cost_raw = self.po.solve(0)
                else:
                    step_target = max(1, int(step_count or 1))
                    self._solving_total_steps = step_target
                    if step_target == 1:
                        _set_status("Solving 1 step")
                    else:
                        _set_status(f"Solving 0 / {step_target} steps")
                    last_cost_raw = self.po.solve(step_target)

                try:
                    last_cost_value = float(last_cost_raw)
                except Exception:
                    last_cost_value = None

                if last_cost_value is not None and prev_cost_local is not None:
                    try:
                        diff_val = last_cost_value - float(prev_cost_local)
                        if abs(diff_val) <= self._convergence_tolerance:
                            converged_successfully = True
                    except Exception:
                        diff_val = None

                if last_cost_value is not None:
                    prev_cost_local = last_cost_value
            finally:
                after_total: Optional[int] = None
                try:
                    after_total = int(self.po.get_total_iterations())
                except Exception:
                    after_total = None

                done = 0
                if after_total is not None and before_total is not None:
                    int_delta = after_total - before_total
                    if int_delta > 0:
                        done = int_delta
                        self._total_solve_steps = after_total
                if done == 0:
                    if not until_converged:
                        done = max(1, int(step_count or 1))
                    else:
                        done = 1
                    self._total_solve_steps += done

                if done <= 0:
                    done = 1
                self._solving_active = False
                self._solving_total_steps = 0
                self._solving_done_steps = 0
                trigger_auto_loop = False

                updated_prev_cost = None
                if last_cost_value is not None and np.isfinite(last_cost_value):
                    updated_prev_cost = last_cost_value
                elif prev_cost_local is not None:
                    try:
                        prev_val = float(prev_cost_local)
                        if np.isfinite(prev_val):
                            updated_prev_cost = prev_val
                    except Exception:
                        updated_prev_cost = None
                if updated_prev_cost is not None:
                    self._prev_cost = updated_prev_cost

                cost_now = updated_prev_cost if updated_prev_cost is not None else float('-1')
                diff_str = f"{diff_val:.9g}" if diff_val is not None else "NA"
                done = max(1, done)
                done_str = f"step{'s' if done != 1 else ''}"
                status_parts = [
                    f"Solver done {done} {done_str}",
                    f"cost: {cost_now:.9g}",
                    f"cost diff: {diff_str}",
                    f"total steps: {self._total_solve_steps}",
                ]
                if until_converged:
                    status_parts[0] = f"{status_parts[0]} (solve until convergence)"
                if converged_successfully:
                    status_parts.append("converged successfully")
                elif until_converged:
                    status_parts.append("used PoseOptimizer max iterations")
                status_lines = []
                if status_parts:
                    first = status_parts[0]
                    if len(status_parts) >= 4:
                        first = f"{first} | {status_parts[3]}"
                    status_lines.append(first)
                if len(status_parts) >= 3:
                    status_lines.append(f"{status_parts[1]} | {status_parts[2]}")
                if len(status_parts) > 4:
                    status_lines.extend(status_parts[4:])
                status_message = "\n".join(status_lines)
                _set_status(status_message)

                with lock:
                    self._solve_running = False
                    if self._pending_auto_loop:
                        self._pending_auto_loop = False
                        trigger_auto_loop = True

                self._running_until_converged = False
                if trigger_auto_loop:
                    threading.Thread(
                        target=run_auto_loop_detection,
                        daemon=True,
                    ).start()

        def save_trajectory():
            with lock:
                if self._solve_running:
                    return
                self._solve_running = True
            try:
                target_path = self._next_output_path()
                msg = f"Saving optimized trajectory to {target_path} ..."
                print(f"[viz] {msg}")
                _set_status(msg)
                try:
                    self.po.save(target_path)
                except Exception as exc:
                    err_msg = f"Failed to save optimized trajectory: {exc}"
                    print(f"[viz] {err_msg}")
                    _set_status(err_msg)
                else:
                    done_msg = f"Saved optimized trajectory to {target_path}"
                    print(f"[viz] {done_msg}")
                    _set_status(done_msg)
            finally:
                with lock:
                    self._solve_running = False

        def run_auto_loop_detection() -> None:
            with lock:
                if self._solve_running:
                    print("[viz] Auto-loop detection skipped: solver is busy", flush=True)
                    _set_status("Auto-loop detection skipped: solver is busy")
                    return
                self._solve_running = True
            try:
                cell_size_m = self.loop_cell_size_m
                if cell_size_m is None or cell_size_m <= 0:
                    cell_size_m = self._auto_loop_cell_size_m()
                    self.loop_cell_size_m = cell_size_m

                status = "Running auto loop closure ..."
                print(f"[viz] {status}", flush=True)
                _set_status(status)

                constraints_before = list(self.po.get_constraints())
                existing_ids = set()
                existing_pairs = set()
                for constraint in constraints_before:
                    cid = self._constraint_id(constraint)
                    if cid is not None:
                        existing_ids.add(cid)
                    pair = self._pose_to_pose_pair_key(constraint)
                    if pair is not None:
                        existing_pairs.add(pair)

                raw_added = int(self.po.add_relative_loop_constraints(
                    self.loop_min_distance_m, float(cell_size_m),
                    float(self.loop_icp_threshold)))

                constraints_after = list(self.po.get_constraints())
                removed_duplicates = 0
                unique_added = 0
                for constraint in constraints_after:
                    cid = self._constraint_id(constraint)
                    if cid is None or cid in existing_ids:
                        continue
                    pair = self._pose_to_pose_pair_key(constraint)
                    if pair is None:
                        continue
                    if pair in existing_pairs:
                        try:
                            self.po.remove_constraint(cid)
                            removed_duplicates += 1
                        except Exception:
                            pass
                    else:
                        existing_pairs.add(pair)
                        unique_added += 1

                self._update_all()
                self.viz.update()

                msg = (f"Auto-loop done: +{unique_added} constraints "
                       f"(raw={raw_added}, removed_duplicates={removed_duplicates})")
                print(f"[viz] {msg}", flush=True)
                _set_status(msg)
            except Exception as exc:
                err_msg = f"Auto-loop detection failed: {exc}"
                print(f"[viz] {err_msg}", flush=True)
                _set_status(err_msg)
            finally:
                with lock:
                    self._solve_running = False

        def run_align_with_gravity() -> None:
            with lock:
                if self._solve_running:
                    msg = "Gravity alignment skipped: solver is busy"
                    print(f"[viz] {msg}", flush=True)
                    _set_status(msg)
                    return
                self._solve_running = True
            try:
                msg = "Aligning trajectory with gravity ..."
                print(f"[viz] {msg}", flush=True)
                _set_status(msg)

                logged_skip_reason = False
                if (self._gravity_view_pose_cache is None or
                        self._gravity_view_pose_cache_valid is False):
                    pose, pose_valid = self._compute_gravity_view_pose()
                    self._gravity_view_pose_cache = pose
                    self._gravity_view_pose_cache_valid = pose_valid
                    if not pose_valid:
                        logged_skip_reason = True
                pose = self._gravity_view_pose_cache
                pose_valid = bool(self._gravity_view_pose_cache_valid)
                if (pose_valid and pose.shape == (4, 4) and
                        not np.allclose(pose, np.eye(4), atol=1e-9)):
                    self._gravity_view_pose = _as_f_pose(pose)
                    self.viz.camera.set_yaw(0.0)
                    self.viz.camera.set_roll(0.0)
                    self.viz.camera.set_view_offset((0.0, 0.0, 0.0))
                    self._camera_fitted = False
                    self._update_all()
                    done_msg = "Gravity alignment applied"
                else:
                    if pose_valid:
                        print("[viz] Gravity alignment skipped: already aligned "
                              "with gravity", flush=True)
                        logged_skip_reason = True
                        done_msg = "Gravity alignment skipped"
                    else:
                        done_msg = "Gravity alignment skipped"
                self.viz.update()
                if not logged_skip_reason:
                    print(f"[viz] {done_msg}", flush=True)
                _set_status(done_msg)
            except Exception as exc:
                err_msg = f"Gravity alignment failed: {exc}"
                print(f"[viz] {err_msg}", flush=True)
                _set_status(err_msg)
            finally:
                with lock:
                    self._solve_running = False

        def on_key(ctx, key, mods):
            mods = int(mods)

            if key == ord('P') and mods == 0:
                self._cloud_pt_size = min(10.0, self._cloud_pt_size + 1.0)
                _apply_point_size()
                _set_status(f"Point cloud size: {self._cloud_pt_size:.1f}")
                return True
            if key == ord('P') and mods == MOD_SHIFT:
                self._cloud_pt_size = max(1.0, self._cloud_pt_size - 1.0)
                _apply_point_size()
                _set_status(f"Point cloud size: {self._cloud_pt_size:.1f}")
                return True
            if key == KEY_R and mods == 0:
                new_state = not self._raw_trajectory_visible
                self._set_raw_trajectory_visible(new_state)
                state = "ON" if new_state else "OFF"
                message = f"Raw trajectory: {state}"
                _set_status(message)
                return True
            if key == KEY_N:
                if (mods & MOD_SHIFT) and (mods & MOD_CTRL):
                    threading.Thread(
                        target=run_steps,
                        kwargs={"until_converged": True},
                        daemon=True,
                    ).start()
                    return True
                steps = 10 if (mods & MOD_SHIFT) else 1
                threading.Thread(target=run_steps, args=(steps,), daemon=True).start()
                return False
            if key == KEY_G and mods == 0:
                threading.Thread(target=run_align_with_gravity, daemon=True).start()
                return True
            if key == KEY_H and mods == 0:
                _toggle_sampled_clouds()
                return True
            if key == KEY_S and (mods & MOD_CTRL):
                threading.Thread(target=save_trajectory, daemon=True).start()
                return True
            if key == KEY_T:
                self.show_constraint_labels = not self.show_constraint_labels
                for cid in list(self.constraint_labels.keys()):
                    label = self.constraint_labels.pop(cid)
                    try:
                        self.viz.remove(label)
                    except Exception:
                        pass
                self._update_constraints()
                self.viz.update()
                return True
            if key == KEY_C and mods == 0:
                self.show_pose_to_pose_clouds = not self.show_pose_to_pose_clouds
                state = "ON" if self.show_pose_to_pose_clouds else "OFF"
                _set_status(f"Pose-to-pose clouds: {state}")
                self._update_constraints()
                self.viz.update()
                return True
            if key == KEY_O:
                self.show_node_orientations = not self.show_node_orientations
                state = "ON" if self.show_node_orientations else "OFF"
                _set_status(f"Node direction arrows: {state}")
                self._update_node_orientations()
                self.viz.update()
                return True
            if key == KEY_X and mods == 0:
                self.show_abs_pose_orientations = not self.show_abs_pose_orientations
                state = "ON" if self.show_abs_pose_orientations else "OFF"
                _set_status(f"Absolute pose axes: {state}")
                self._update_abs_pose_orientations()
                self.viz.update()
                return True
            if key == KEY_L and mods == 0:
                with lock:
                    if self._solve_running:
                        self._pending_auto_loop = True
                        queued_msg = (
                            "Auto-loop queued; will start after solver finishes"
                        )
                        print(f"[viz] {queued_msg}", flush=True)
                        _set_status(queued_msg)
                        return True
                threading.Thread(target=run_auto_loop_detection, daemon=True).start()
                return True
            if key == KEY_A and (mods & MOD_CTRL):
                self._select_all_constraints()
                return True
            if key == KEY_U and mods == 0:
                self._remove_selected_constraints()
                return True
            return False

        self.viz.push_key_handler(on_key)
        self._viz_module.add_default_controls(self.viz)
        self._set_status = _set_status

    # --- helper/updater routines --------------------------------------
    def _setup_mouse_pan(self) -> None:
        self._right_drag_active = False

        def _mouse_button_handler(
            ctx: WindowCtx,
            button: MouseButton,
            event: MouseButtonEvent,
            mods: EventModifierKeys,
        ) -> bool:
            try:
                if button == self._viz_module.MouseButton.MOUSE_BUTTON_RIGHT:
                    if event == self._viz_module.MouseButtonEvent.MOUSE_BUTTON_PRESSED:
                        self._right_drag_active = True
                    elif event == self._viz_module.MouseButtonEvent.MOUSE_BUTTON_RELEASED:
                        self._right_drag_active = False
                    return False
                if button == self._viz_module.MouseButton.MOUSE_BUTTON_LEFT:
                    if event == self._viz_module.MouseButtonEvent.MOUSE_BUTTON_PRESSED:
                        self._left_click_start = (float(ctx.mouse_x), float(ctx.mouse_y))
                        self._left_click_moved = False
                        return True
                    if event == self._viz_module.MouseButtonEvent.MOUSE_BUTTON_RELEASED:
                        was_click = (
                            self._left_click_start is not None and
                            not self._left_click_moved
                        )
                        self._left_click_start = None
                        self._left_click_moved = False
                        if not was_click:
                            return True
                        if self._is_remove_button_click(ctx):
                            self._remove_selected_constraints()
                            return False
                        self._pick_constraints_at(
                            int(ctx.mouse_x), int(ctx.mouse_y), int(mods)
                        )
                        return False
            except Exception:
                pass
            return True

        def _mouse_pos_handler(ctx: WindowCtx, x: float, y: float) -> bool:
            if self._left_click_start is not None:
                dx = float(x - self._left_click_start[0])
                dy = float(y - self._left_click_start[1])
                if dx * dx + dy * dy > (self._left_click_threshold_px ** 2):
                    self._left_click_moved = True
            if self._right_drag_active:
                try:
                    dx = float(x - ctx.mouse_x)
                    dy = float(y - ctx.mouse_y)
                    w = float(ctx.window_width)
                    h = float(ctx.window_height)
                    diag = max(1.0, (w * w + h * h) ** 0.5)
                    self.viz.camera.dolly_xy(2.0 * dx / diag, 2.0 * dy / diag)
                    self.viz.update()
                    return False
                except Exception:
                    return True
            return True

        self.viz.push_mouse_button_handler(_mouse_button_handler)
        self.viz.push_mouse_pos_handler(_mouse_pos_handler)

    def _fit_camera_to_points(
        self,
        xyz: np.ndarray,
        margin: float = 1.4,
        pitch_deg: float = -60.0,
    ) -> None:
        if xyz.size == 0:
            return
        center = xyz.mean(axis=0)
        r = float(np.max(np.linalg.norm(xyz - center, axis=1)))
        fov_deg = self.viz.camera.get_fov()
        half_diag = np.deg2rad(max(1e-3, fov_deg / 2.0))
        d = max(1.0, (r * margin) / np.tan(half_diag))
        logd = 100.0 * np.log(d / 50.0)

        T = np.eye(4, dtype=np.float64)
        T[:3, 3] = center.astype(np.float64)
        target_pose = self._gravity_view_pose @ T
        self.viz.camera.set_target(
            np.asfortranarray(np.linalg.inv(target_pose)))
        self.viz.camera.set_pitch(pitch_deg)
        self.viz.camera.set_dolly(logd)

    def _compute_gravity_view_pose(self) -> Tuple[np.ndarray, bool]:
        identity = np.eye(4, dtype=np.float64)
        valid_mask = 0x01
        max_probe_frames = 10
        probed = 0
        source = None

        def log_skip(reason: str) -> None:
            print(f"[viz] Gravity alignment skipped: {reason}", flush=True)

        try:
            if not self.input_source_uri:
                log_skip("no input source URI provided")
                return identity, False
            source = open_source(self.input_source_uri, collate=False)
            for frame_sets in source:
                for frame in frame_sets:
                    if probed >= max_probe_frames:
                        log_skip(
                            f"no IMU frame found in first {max_probe_frames} frames")
                        return identity, False
                    if frame is None:
                        continue
                    probed += 1
                    if not frame.has_field("IMU_STATUS") or not frame.has_field("IMU_ACC"):
                        continue

                    imu_status = np.asarray(frame.field("IMU_STATUS")).reshape(-1)
                    imu_acc = np.asarray(frame.field("IMU_ACC"), dtype=np.float64)
                    if imu_acc.ndim != 2 or imu_acc.shape[1] != 3:
                        log_skip("IMU_ACC has invalid shape")
                        return identity, False
                    if imu_acc.shape[0] != imu_status.shape[0]:
                        log_skip("IMU_STATUS and IMU_ACC size mismatch")
                        return identity, False

                    valid = ((imu_status.astype(np.uint64) & valid_mask) != 0)
                    if not np.any(valid):
                        log_skip("first IMU frame has no valid IMU samples")
                        return identity, False

                    acc = imu_acc[valid]
                    finite = np.all(np.isfinite(acc), axis=1)
                    if not np.any(finite):
                        log_skip("first IMU frame has no finite IMU samples")
                        return identity, False
                    acc = acc[finite]

                    sensor_info = getattr(frame, "sensor_info", None)
                    if sensor_info is not None:
                        extrinsic = np.asarray(
                            sensor_info.sensor_to_body, dtype=np.float64
                        )[:3, :3]
                        imu_to_sensor = np.asarray(
                            sensor_info.imu_to_sensor_transform, dtype=np.float64
                        )[:3, :3]
                        acc = (extrinsic @ imu_to_sensor @ acc.T).T

                    avg_acc = np.mean(acc, axis=0)
                    if (not np.all(np.isfinite(avg_acc)) or
                            np.linalg.norm(avg_acc) <= 1e-6):
                        log_skip("first IMU frame has invalid acceleration vector")
                        return identity, False
                    rot = np.asarray(
                        get_rot_matrix_to_align_to_gravity(
                            float(avg_acc[0]),
                            float(avg_acc[1]),
                            float(avg_acc[2]),
                        ),
                        dtype=np.float64,
                    )
                    pose_mat = np.eye(4, dtype=np.float64)
                    pose_mat[:3, :3] = rot
                    pose = _as_f_pose(pose_mat)
                    print("[viz] Gravity alignment IMU source: "
                          f"using frame #{probed} with "
                          f"{int(acc.shape[0])} valid IMU samples", flush=True)
                    return pose, True
        except Exception as exc:
            log_skip(f"failed to read IMU data: {exc}")
            return identity, False
        finally:
            if source is not None:
                try:
                    source.close()
                except Exception:
                    pass

        if probed == 0:
            log_skip("input source contains no frames")
            return identity, False
        log_skip("no frame with IMU_STATUS/IMU_ACC found")
        return identity, False

    @staticmethod
    def _make_line_segments(xyz: np.ndarray) -> np.ndarray:
        pts = np.asarray(xyz, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[0] < 2:
            return np.zeros((0, 3), dtype=np.float32)
        segs = np.empty((2 * (pts.shape[0] - 1), 3), dtype=np.float32)
        segs[0::2] = pts[:-1]
        segs[1::2] = pts[1:]
        return segs

    @staticmethod
    def _constraint_id(constraint: Any) -> Optional[int]:
        try:
            return int(constraint.get_constraint_id())
        except (AttributeError, TypeError):
            return None

    @staticmethod
    def _pose_to_pose_pair_key(constraint: Any) -> Optional[Tuple[int, int]]:
        try:
            ts1 = int(constraint.timestamp1)
            ts2 = int(constraint.timestamp2)
            _ = constraint.relative_pose  # Ensure it exists
        except (AttributeError, TypeError, ValueError):
            return None
        if ts1 <= 0 or ts2 <= 0:
            return None
        if ts1 <= ts2:
            return (ts1, ts2)
        return (ts2, ts1)

    def _auto_loop_cell_size_m(self) -> float:
        from .source_mapping import auto_loop_cell_size_m

        return auto_loop_cell_size_m(self.po)

    @staticmethod
    def _pose_matrix(pose: np.ndarray) -> np.ndarray:
        return np.asarray(pose, dtype=np.float64)

    def _next_output_path(self) -> str:
        base_path = self.output_osf_filename
        base, ext = os.path.splitext(base_path)
        candidate = base_path
        idx = 1
        while os.path.exists(candidate):
            candidate = f"{base}{idx}{ext}"
            idx += 1
        return candidate

    def _update_help_label(self) -> None:
        if not self._help_label:
            return
        lines = list(self._help_lines)
        if self._status_text:
            lines += ["", self._status_text]
        try:
            self._help_label.set_text("\n".join(lines))
        except Exception:
            pass

    def _update_remove_button_label(self) -> None:
        if not self._remove_button_label:
            return
        selected = len(self.selected_constraint_ids)
        label_text = f"[U] Remove selected constraints ({selected})"
        try:
            self._remove_button_label.set_text(label_text)
            if selected > 0:
                self._remove_button_label.set_rgba((1.0, 0.45, 0.45, 1.0))
            else:
                self._remove_button_label.set_rgba((0.75, 0.75, 0.75, 0.9))
        except Exception:
            pass

    def _is_remove_button_click(self, ctx: WindowCtx) -> bool:
        width = max(1.0, float(ctx.window_width))
        height = max(1.0, float(ctx.window_height))
        nx = float(ctx.mouse_x) / width
        ny = float(ctx.mouse_y) / height
        anchor_x, anchor_y = self._remove_button_anchor
        size_x, size_y = self._remove_button_size
        x0 = anchor_x - size_x
        x1 = anchor_x
        y0 = anchor_y
        y1 = anchor_y + size_y
        return x0 <= nx <= x1 and y0 <= ny <= y1

    @staticmethod
    def _constraint_pick_callback(
        cid: int,
        picked_ids: set[int],
        _pts: Any,
    ) -> None:
        if cid > 0:
            picked_ids.add(cid)

    def _apply_constraint_selection_hits(self, hits: set[int], mods: int) -> None:
        with self._lock:
            prev = set(self.selected_constraint_ids)
            mod_shift = int(self._viz_module.EventModifierKeys.MOD_SHIFT)
            mod_ctrl = int(self._viz_module.EventModifierKeys.MOD_CONTROL)

            if mods & mod_ctrl:
                for cid in hits:
                    if cid in self.selected_constraint_ids:
                        self.selected_constraint_ids.remove(cid)
                    else:
                        self.selected_constraint_ids.add(cid)
            elif mods & mod_shift:
                self.selected_constraint_ids.update(hits)
            else:
                if hits and self.selected_constraint_ids == set(hits):
                    self.selected_constraint_ids.clear()
                else:
                    self.selected_constraint_ids = set(hits)

            self.selected_constraint_ids.intersection_update(
                set(self._constraint_segments.keys())
            )
            if self.selected_constraint_ids == prev:
                return

        if self.selected_constraint_ids:
            selected_str = ", ".join(
                str(cid) for cid in sorted(self.selected_constraint_ids)
            )
            self._set_status(f"Selected constraint ids: {selected_str}")
        else:
            self._set_status("No constraint selected")
        self._update_constraints()
        self._update_remove_button_label()
        self.viz.update()

    def _pick_constraints_at(self, x: int, y: int, mods: int) -> None:
        with self._lock:
            segments = dict(self._constraint_segments)
        self._update_constraint_pick_clouds(segments)
        try:
            self.viz.update()
        except Exception:
            pass
        self._pick_hits.clear()
        try:
            self.viz.pick(int(x) - 6, int(y) - 6, int(x) + 6, int(y) + 6)
            if not self._pick_hits:
                self.viz.pick(int(x) - 14, int(y) - 14, int(x) + 14, int(y) + 14)
            if not self._pick_hits:
                self.viz.pick(int(x) - 24, int(y) - 24, int(x) + 24, int(y) + 24)
            if not self._pick_hits:
                self.viz.pick(int(x) - 36, int(y) - 36, int(x) + 36, int(y) + 36)
            if not self._pick_hits:
                self.viz.pick(int(x), int(y), int(x), int(y))
        except Exception:
            pass
        finally:
            self._clear_constraint_pick_clouds()
            try:
                self.viz.update()
            except Exception:
                pass

        self._apply_constraint_selection_hits(set(self._pick_hits), mods)

    def _clear_constraint_pick_clouds(self) -> None:
        for cid in list(self.constraint_pick_clouds.keys()):
            cloud = self.constraint_pick_clouds.pop(cid)
            try:
                cloud.clear_on_select()
            except Exception:
                pass
            try:
                self.viz.remove(cloud)
            except Exception:
                pass

    def _remove_selected_constraints(self) -> None:
        with self._lock:
            selected_ids = sorted(int(cid) for cid in self.selected_constraint_ids if cid > 0)
        if not selected_ids:
            self._set_status("No selected constraints to remove")
            return

        removed_ids: List[int] = []
        failed_ids: List[int] = []
        for cid in selected_ids:
            try:
                self.po.remove_constraint(int(cid))
                removed_ids.append(cid)
            except Exception:
                failed_ids.append(cid)

        with self._lock:
            self.selected_constraint_ids.difference_update(set(removed_ids))
        self._update_all()
        self._update_remove_button_label()
        self.viz.update()

        if failed_ids:
            failed_str = ", ".join(str(cid) for cid in failed_ids)
            msg = (
                f"Removed {len(removed_ids)} / {len(selected_ids)} selected constraints. "
                f"Failed ids: {failed_str}"
            )
        else:
            msg = f"Removed {len(removed_ids)} selected constraints"
        self._set_status(msg)

    def _select_all_constraints(self) -> None:
        with self._lock:
            self._update_constraints()
            all_ids = {
                int(cid) for cid in self._constraint_segments.keys() if int(cid) > 0
            }

            prev = set(self.selected_constraint_ids)
            self.selected_constraint_ids = set(all_ids)

        if self.selected_constraint_ids:
            msg = f"Selected all constraints ({len(self.selected_constraint_ids)})"
        else:
            msg = "No constraints available to select"
        if self.selected_constraint_ids != prev or not self.selected_constraint_ids:
            self._set_status(msg)

        self._update_constraints()
        self._update_remove_button_label()
        self.viz.update()

    @staticmethod
    def _make_constraint_pick_points(
        p0: np.ndarray,
        p1: np.ndarray,
        sample_count: int,
    ) -> np.ndarray:
        count = max(2, int(sample_count))
        a = np.asarray(p0, dtype=np.float32).reshape(3)
        b = np.asarray(p1, dtype=np.float32).reshape(3)
        t = np.linspace(0.0, 1.0, count, dtype=np.float32)
        pts = (1.0 - t[:, None]) * a[None, :] + t[:, None] * b[None, :]
        return np.asarray(pts, dtype=np.float32)

    def _update_constraint_pick_clouds(
        self,
        segments_by_id: Dict[int, Tuple[np.ndarray, np.ndarray]],
    ) -> None:
        active_ids = set(int(cid) for cid in segments_by_id.keys() if int(cid) > 0)
        for cid in list(self.constraint_pick_clouds.keys()):
            if cid in active_ids:
                continue
            cloud = self.constraint_pick_clouds.pop(cid)
            try:
                cloud.clear_on_select()
            except Exception:
                pass
            try:
                self.viz.remove(cloud)
            except Exception:
                pass

        for cid, (p0, p1) in segments_by_id.items():
            if int(cid) <= 0:
                continue
            points = self._make_constraint_pick_points(
                p0, p1, self._pick_samples_per_constraint
            )
            cloud = self.constraint_pick_clouds.get(int(cid))  # type: ignore[assignment]
            if cloud is None or cloud.size != points.shape[0]:
                if cloud is not None:
                    try:
                        self.viz.remove(cloud)
                    except Exception:
                        pass
                cloud = self._viz_module.Cloud(points.shape[0])
                cloud.set_point_size(8.0)
                cloud.set_key(
                    np.tile(
                        np.array([1.0, 1.0, 1.0, 0.02], dtype=np.float32),
                        (points.shape[0], 1),
                    )
                )
                cloud.set_on_select(
                    partial(
                        PoseOptimizerViz._constraint_pick_callback,
                        int(cid),
                        self._pick_hits,
                    )
                )
                self.viz.add(cloud)
                self.constraint_pick_clouds[int(cid)] = cloud
            cloud.set_xyz(points)

    def _set_raw_trajectory_visible(self, visible: bool) -> None:
        current = getattr(self, "_raw_trajectory_visible", True)
        if visible == current:
            return
        if visible:
            self.viz.add(self.raw_cloud)
            self.viz.add(self.raw_line)
        else:
            self.viz.remove(self.raw_cloud)
            self.viz.remove(self.raw_line)
        self._raw_trajectory_visible = visible
        self.viz.update()

    def _ensure_node_cloud(self, node):
        ts = int(node.ts)
        if ts in self.node_clouds:
            cloud = self.node_clouds[ts]
        else:
            pts = np.ascontiguousarray(
                node.downsampled_pts, dtype=np.float32)
            if pts.size == 0:
                cloud = self._viz_module.Cloud(1)
                cloud.set_xyz(np.zeros((1, 3), dtype=np.float32))
                cloud.set_point_size(self._cloud_pt_size)
                cloud.set_key(np.tile(self.node_rgba, (1, 1)))
            else:
                cloud = self._viz_module.Cloud(pts.shape[0])
                cloud.set_xyz(pts)
                cloud.set_point_size(self._cloud_pt_size)
                cloud.set_key(np.tile(self.node_rgba, (pts.shape[0], 1)))
            self.viz.add(cloud)
            self.node_clouds[ts] = cloud
        cloud.set_pose(_as_f_pose(node.get_pose()))
        return cloud

    @staticmethod
    def _node_has_points(node, attr: str) -> bool:
        try:
            pts = getattr(node, attr)
            return pts is not None and np.asarray(pts).size > 0
        except AttributeError:
            return False

    @staticmethod
    def _first_local_point(node, attr: str) -> Optional[np.ndarray]:
        try:
            pts = getattr(node, attr)
            arr = np.asarray(pts, dtype=np.float64)
        except AttributeError:
            return None
        if arr.ndim == 1:
            if arr.size == 0:
                return None
            return arr.reshape(-1, 3)[0]
        if arr.ndim >= 2:
            if arr.size == 0:
                return None
            return arr.reshape(-1, 3)[0]
        return None

    def _keyframe_pose_stack(self) -> np.ndarray:
        poses: List[np.ndarray] = []
        for ts in getattr(self, "_keyframe_ts_base", []):
            node = None
            try:
                node = self.po.get_node(int(ts))
            except Exception:
                node = None
            if not node:
                continue
            try:
                pose = np.asarray(node.get_pose(), dtype=np.float64)
            except Exception:
                continue
            if pose.shape != (4, 4):
                continue
            poses.append(pose)
        if not poses:
            return np.zeros((0, 4, 4), dtype=np.float64)
        return np.stack(poses, axis=0)

    def _marker(self, key: Tuple[int, str], p_local: np.ndarray, pose: np.ndarray,
                color: Tuple[float, float, float, float], size: float = 4.0) -> None:
        if key in self.marker_clouds:
            cl = self.marker_clouds[key]
            cl.set_xyz(p_local.reshape(1, 3).astype(np.float32))
            cl.set_pose(_as_f_pose(pose))
        else:
            cl = self._viz_module.Cloud(1)
            cl.set_xyz(p_local.reshape(1, 3).astype(np.float32))
            cl.set_point_size(size)
            cl.set_key(np.array([color], dtype=np.float32))
            cl.set_pose(_as_f_pose(pose))
            self.viz.add(cl)
            self.marker_clouds[key] = cl

    def _set_constraint_label(self, cid: int, text: str,
                              position: np.ndarray,
                              extra_lines: Optional[List[str]] = None) -> None:
        if not self.show_constraint_labels:
            if cid in self.constraint_labels:
                label = self.constraint_labels.pop(cid)
                self.viz.remove(label)
            return
        pos = np.asarray(position, dtype=np.float64).reshape(-1)
        if pos.size < 3 or not np.all(np.isfinite(pos[:3])):
            return
        is_selected = cid in self.selected_constraint_ids
        title = text + (" [selected]" if is_selected else "")
        lines = [title, f"id: {cid}"]
        if extra_lines:
            lines.extend(extra_lines)
        label_text = "\n".join(lines)
        constraint_label: Optional[Label] = self.constraint_labels.get(cid)
        if constraint_label is None:
            constraint_label = self._viz_module.Label(label_text, float(pos[0]), float(pos[1]), float(pos[2]))
            if is_selected:
                constraint_label.set_rgba(self.label_selected_rgba)
            else:
                constraint_label.set_rgba(self.label_rgba)
            constraint_label.set_scale(4.0)
            self.viz.add(constraint_label)
            self.constraint_labels[cid] = constraint_label
        else:
            constraint_label.set_text(label_text)
            constraint_label.set_position(float(pos[0]), float(pos[1]), float(pos[2]))
            constraint_label.set_scale(4.0)
            if is_selected:
                constraint_label.set_rgba(self.label_selected_rgba)
            else:
                constraint_label.set_rgba(self.label_rgba)

    def _draw_ptp(self, cid: int, c, active_label_ids: set[int]) -> Tuple[List[np.ndarray], Optional[Tuple[int, int]]]:
        segs: List[np.ndarray] = []
        pair_ts: Optional[Tuple[int, int]] = None

        if not isinstance(c, mapping.PointToPointConstraint):
            return segs, pair_ts
        ts1 = int(c.timestamp1)
        ts2 = int(c.timestamp2)
        if ts1 <= 0 or ts2 <= 0:
            return segs, pair_ts

        n1 = self.po.get_node(ts1)
        n2 = self.po.get_node(ts2)

        if not n1 or not n2:
            return segs, pair_ts

        self._ensure_node_cloud(n1)
        self._ensure_node_cloud(n2)

        p1_local = self._first_local_point(n1, 'ptp_constraint_pt')
        if p1_local is None:
            return segs, pair_ts
        p2_local = self._first_local_point(n2, 'ptp_constraint_pt')
        if p2_local is None:
            return segs, pair_ts

        T1 = self._pose_matrix(n1.get_pose())
        T2 = self._pose_matrix(n2.get_pose())

        self._marker((cid, "ptp1"), p1_local, T1,
                     tuple(self.pose_marker_red.tolist()))
        self._marker((cid, "ptp2"), p2_local, T2,
                     tuple(self.pose_marker_blue.tolist()))

        p1 = (T1[:3, :3] @ p1_local[:3]) + T1[:3, 3]
        p2 = (T2[:3, :3] @ p2_local[:3]) + T2[:3, 3]
        segs += [p1.astype(np.float32), p2.astype(np.float32)]

        extra_lines: List[str] = []
        if self.show_constraint_timestamps:
            extra_lines.append(f"ts: {ts1}")
            extra_lines.append(f"ts: {ts2}")
        self._set_constraint_label(cid, "PointToPoint", (p1 + p2) * 0.5,
                                   extra_lines=extra_lines)
        active_label_ids.add(cid)

        pair_ts = (int(n1.ts), int(n2.ts))
        return segs, pair_ts

    def _draw_abs_point(self, cid: int, c, active_label_ids: set[int]) -> List[np.ndarray]:
        segs: List[np.ndarray] = []

        if not isinstance(c, mapping.AbsolutePointConstraint):
            return segs
        ts = int(c.timestamp)
        if ts <= 0:
            return segs

        n = self.po.get_node(ts)

        if not n:
            return segs

        self._ensure_node_cloud(n)

        p_local = self._first_local_point(n, 'ap_constraint_pt')
        if p_local is None:
            return segs

        T = self._pose_matrix(n.get_pose())
        self._marker((cid, "ap"), p_local, T, (1.0, 1.0, 0.0, 1.0))
        tgt = np.asarray(c.absolute_position, dtype=np.float64).reshape(3)
        p = (T[:3, :3] @ p_local[:3]) + T[:3, 3]
        segs += [p.astype(np.float32), tgt.astype(np.float32)]
        self._set_constraint_label(cid, "AbsolutePoint", (p + tgt) * 0.5)
        active_label_ids.add(cid)
        return segs

    def _draw_abs_pose(self, cid: int, c, active_label_ids: set[int]) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        segs: List[np.ndarray] = []
        targets: List[np.ndarray] = []
        if not isinstance(c, mapping.AbsolutePoseConstraint):
            return segs, targets
        ts = int(c.timestamp)
        if ts <= 0:
            return segs, targets

        n = self.po.get_node(ts)
        if not n:
            return segs, targets

        Tn = self._pose_matrix(n.get_pose())
        p_now = Tn[:3, 3].astype(np.float32)
        Tt = self._pose_matrix(c.pose)
        p_tgt = Tt[:3, 3].astype(np.float32)
        if hasattr(self, "_abs_pose_axes_buffer"):
            weight = getattr(c, 'rotation_weight', None)
            if weight is None or not np.isclose(weight, 0.0):
                self._abs_pose_axes_buffer.append(Tt)
        segs += [p_tgt, p_now]
        targets.append(p_tgt)
        origin = np.zeros(3, dtype=np.float64)
        self._marker((cid, "abs_pose_node"), origin, Tn,
                     tuple(self.pose_marker_red.tolist()), size=5.0)
        extra_lines = []
        if self.show_constraint_timestamps:
            ts_label = f"ts: {ts}"
            extra_lines.append(ts_label)
        self._set_constraint_label(cid, "AbsolutePose", (p_now + p_tgt) * 0.5,
                                   extra_lines=extra_lines)
        active_label_ids.add(cid)
        return segs, targets

    def _update_node_orientations(self) -> None:
        if not self.show_node_orientations:
            for axis_lines in self.node_axis_lines:
                axis_lines.set_points(np.zeros((0, 3), dtype=np.float32))
            return

        try:
            if (self.sampling_mode == mapping.SamplingMode.KEY_FRAMES and
                    getattr(self, "_keyframe_ts_base", [])):
                poses_np = self._keyframe_pose_stack()
            else:
                poses = self.po.get_poses(self.sampling_mode)
                poses_np = np.asarray(poses, dtype=np.float64)
        except Exception:
            for axis_lines in self.node_axis_lines:
                axis_lines.set_points(np.zeros((0, 3), dtype=np.float32))
            return
        if poses_np.ndim != 3 or poses_np.shape[0] == 0:
            for axis_lines in self.node_axis_lines:
                axis_lines.set_points(np.zeros((0, 3), dtype=np.float32))
            return

        origins = poses_np[:, :3, 3]
        axes = [poses_np[:, :3, axis] for axis in range(3)]

        for axis_idx, axis_lines in enumerate(self.node_axis_lines):
            headings = axes[axis_idx]
            norms = np.linalg.norm(headings, axis=1)
            mask = norms > 1e-6
            if not np.any(mask):
                axis_lines.set_points(np.zeros((0, 3), dtype=np.float32))
                continue

            axis_origins = origins[mask]
            axis_headings = headings[mask]
            axis_norms = norms[mask][:, None]
            scaled = axis_headings / axis_norms
            tips = axis_origins + scaled * float(self._orientation_length)

            segments = np.empty((axis_origins.shape[0] * 2, 3), dtype=np.float32)
            segments[0::2] = axis_origins.astype(np.float32)
            segments[1::2] = tips.astype(np.float32)
            axis_lines.set_points(segments)

    def _update_abs_pose_orientations(self) -> None:
        if not self.show_abs_pose_orientations:
            for axis_lines in self.abs_pose_axis_lines:
                axis_lines.set_points(np.zeros((0, 3), dtype=np.float32))
            return

        if not self._abs_pose_axes_buffer:
            for axis_lines in self.abs_pose_axis_lines:
                axis_lines.set_points(np.zeros((0, 3), dtype=np.float32))
            return

        axis_segments: List[List[np.ndarray]] = [[], [], []]
        length = float(self._orientation_length)

        for pose in self._abs_pose_axes_buffer:
            T = np.asarray(pose, dtype=np.float64)
            if T.shape != (4, 4):
                continue
            origin = T[:3, 3]
            axes = T[:3, :3]
            for axis_idx in range(3):
                start = origin.astype(np.float32)
                end = (origin + axes[:, axis_idx] * length).astype(np.float32)
                axis_segments[axis_idx].extend([start, end])

        for axis_idx, axis_lines in enumerate(self.abs_pose_axis_lines):
            segs = axis_segments[axis_idx]
            if segs:
                axis_lines.set_points(np.asarray(segs, dtype=np.float32))
            else:
                axis_lines.set_points(np.zeros((0, 3), dtype=np.float32))

    def _update_constraints(self) -> None:
        cons = self.po.get_constraints()

        segs_ptp: List[np.ndarray] = []
        segs_abspt: List[np.ndarray] = []
        segs_abspose: List[np.ndarray] = []
        targets: List[np.ndarray] = []
        cloud_color_overrides: Dict[int, np.ndarray] = {}
        cloud_color_priorities: Dict[int, int] = {}
        abs_point_nodes: set[int] = set()
        pose_to_pose_nodes: set[int] = set()
        other_nodes: set[int] = set()
        active_label_ids: set[int] = set()
        active_constraint_ids: set[int] = set()
        segments_by_id: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}

        def _set_cloud_override(ts: int, rgba: np.ndarray, priority: int) -> None:
            if priority < cloud_color_priorities.get(ts, -1):
                return
            cloud_color_overrides[ts] = rgba
            cloud_color_priorities[ts] = priority

        self._abs_pose_axes_buffer = []

        if not self.show_constraint_labels and self.constraint_labels:
            for label in self.constraint_labels.values():
                try:
                    self.viz.remove(label)
                except Exception:
                    pass
            self.constraint_labels.clear()

        for c in cons:
            try:
                cid_local = self._constraint_id(c)
                if cid_local is None:
                    continue
                if cid_local > 0:
                    active_constraint_ids.add(cid_local)

                if isinstance(c, mapping.PoseToPoseConstraint):
                    ts1 = int(c.timestamp1)
                    ts2 = int(c.timestamp2)
                    if ts1 <= 0 or ts2 <= 0:
                        continue
                    n1 = self.po.get_node(ts1)
                    n2 = self.po.get_node(ts2)
                    pose_to_pose_nodes.add(ts1)
                    pose_to_pose_nodes.add(ts2)
                    if self.show_pose_to_pose_clouds:
                        if n1:
                            self._ensure_node_cloud(n1)
                        if n2:
                            self._ensure_node_cloud(n2)
                    if n1 and n2:
                        T1 = self._pose_matrix(n1.get_pose())
                        T2 = self._pose_matrix(n2.get_pose())
                        p1 = T1[:3, 3].astype(np.float32)
                        p2 = T2[:3, 3].astype(np.float32)
                        segs_abspose += [
                            p1,
                            p2,
                        ]
                        origin = np.zeros(3, dtype=np.float64)
                        self._marker((cid_local, "pose1"), origin, T1,
                                     tuple(self.pose_marker_red.tolist()), size=6.0)
                        self._marker((cid_local, "pose2"), origin, T2,
                                     tuple(self.pose_marker_blue.tolist()), size=6.0)
                        if cid_local > 0:
                            segments_by_id[cid_local] = (p1, p2)
                        mid = ((p1 + p2) * 0.5).astype(np.float32)
                        extra_lines = []
                        if self.show_constraint_timestamps:
                            extra_lines.append(f"ts: {ts1}")
                            extra_lines.append(f"ts: {ts2}")
                        self._set_constraint_label(
                            cid_local,
                            "PoseToPose",
                            mid,
                            extra_lines=extra_lines,
                        )
                        active_label_ids.add(cid_local)
                        if self.show_pose_to_pose_clouds:
                            is_selected = cid_local in self.selected_constraint_ids
                            c1 = (self.cloud_selected_a
                                  if is_selected else self.cloud_red)
                            c2 = (self.cloud_selected_b
                                  if is_selected else self.cloud_blue)
                            priority = 2 if is_selected else 1
                            try:
                                _set_cloud_override(int(n1.ts), c1, priority)
                            except Exception:
                                pass
                            try:
                                _set_cloud_override(int(n2.ts), c2, priority)
                            except Exception:
                                pass
                elif isinstance(c, mapping.PointToPointConstraint):
                    segs, pair_ts = self._draw_ptp(cid_local, c, active_label_ids)
                    segs_ptp += segs
                    if cid_local > 0 and len(segs) >= 2:
                        p0 = np.asarray(segs[0], dtype=np.float32).reshape(3)
                        p1 = np.asarray(segs[1], dtype=np.float32).reshape(3)
                        segments_by_id[cid_local] = (p0, p1)
                    if pair_ts is not None:
                        _set_cloud_override(pair_ts[0], self.cloud_red, 1)
                        _set_cloud_override(pair_ts[1], self.cloud_blue, 1)
                        other_nodes.add(pair_ts[0])
                        other_nodes.add(pair_ts[1])
                elif isinstance(c, mapping.AbsolutePointConstraint):
                    segs = self._draw_abs_point(cid_local, c, active_label_ids)
                    segs_abspt += segs
                    if cid_local > 0 and len(segs) >= 2:
                        p0 = np.asarray(segs[0], dtype=np.float32).reshape(3)
                        p1 = np.asarray(segs[1], dtype=np.float32).reshape(3)
                        segments_by_id[cid_local] = (p0, p1)
                    ts = int(c.timestamp)
                    if ts > 0:
                        abs_point_nodes.add(ts)
                        other_nodes.add(ts)
                elif isinstance(c, mapping.AbsolutePoseConstraint):
                    segs, t = self._draw_abs_pose(cid_local, c, active_label_ids)
                    segs_abspose += segs
                    if cid_local > 0 and len(segs) >= 2:
                        p0 = np.asarray(segs[0], dtype=np.float32).reshape(3)
                        p1 = np.asarray(segs[1], dtype=np.float32).reshape(3)
                        segments_by_id[cid_local] = (p0, p1)
                    targets.extend(t)
            except Exception:
                continue

        if segs_ptp:
            self.lines_ptp.set_points(np.asarray(segs_ptp, dtype=np.float32))
        else:
            self.lines_ptp.set_points(np.zeros((0, 3), dtype=np.float32))
        if segs_abspt:
            self.lines_abspt.set_points(np.asarray(segs_abspt, dtype=np.float32))
        else:
            self.lines_abspt.set_points(np.zeros((0, 3), dtype=np.float32))
        if segs_abspose:
            self.lines_abspose.set_points(np.asarray(segs_abspose, dtype=np.float32))
        else:
            self.lines_abspose.set_points(np.zeros((0, 3), dtype=np.float32))

        if targets:
            P = np.asarray(targets, dtype=np.float32)
            if self.targets_cloud.size != P.shape[0]:
                self.viz.remove(self.targets_cloud)
                self.targets_cloud = self._viz_module.Cloud(P.shape[0])
                self.targets_cloud.set_point_size(7.0)
                self.viz.add(self.targets_cloud)
            self.targets_cloud.set_xyz(P)
            self.targets_cloud.set_key(
                np.tile(np.array([1.0, 1.0, 1.0, 1.0], dtype=np.float32), (P.shape[0], 1))
            )
        else:
            if self.targets_cloud.size != 1:
                try:
                    self.viz.remove(self.targets_cloud)
                except Exception:
                    pass
                self.targets_cloud = self._viz_module.Cloud(1)
                self.targets_cloud.set_point_size(7.0)
                self.viz.add(self.targets_cloud)
            self.targets_cloud.set_xyz(np.zeros((1, 3), dtype=np.float32))
            self.targets_cloud.set_key(np.zeros((1, 4), dtype=np.float32))
        self._update_abs_pose_orientations()

        for ts in list(cloud_color_overrides.keys()):
            if ts in abs_point_nodes:
                del cloud_color_overrides[ts]

        desired_visible_nodes = set(other_nodes)
        if self.show_pose_to_pose_clouds:
            desired_visible_nodes.update(pose_to_pose_nodes)

        for ts in list(self.node_clouds.keys()):
            if ts in desired_visible_nodes:
                continue
            cloud = self.node_clouds.pop(ts)
            self.viz.remove(cloud)

        for ts, cloud in self.node_clouds.items():
            if ts not in cloud_color_overrides:
                cloud.set_key(np.tile(self.node_rgba, (cloud.size, 1)))
        for ts, rgba in cloud_color_overrides.items():
            override_cloud: Optional[Cloud] = self.node_clouds.get(ts)
            if override_cloud is not None:
                override_cloud.set_key(
                    np.tile(rgba, (override_cloud.size, 1))
                )

        for cid_local in list(self.constraint_labels.keys()):
            if cid_local not in active_label_ids:
                label = self.constraint_labels.pop(cid_local)
                self.viz.remove(label)

        for marker_key in list(self.marker_clouds.keys()):
            cid_local = int(marker_key[0])
            if cid_local <= 0 or cid_local in active_constraint_ids:
                continue
            cloud = self.marker_clouds.pop(marker_key)
            self.viz.remove(cloud)

        prev_selected = set(self.selected_constraint_ids)
        self.selected_constraint_ids.intersection_update(active_constraint_ids)
        self._constraint_segments = segments_by_id
        self._clear_constraint_pick_clouds()

        selected_segments: List[np.ndarray] = []
        for cid in sorted(self.selected_constraint_ids):
            seg = segments_by_id.get(cid)
            if seg is None:
                continue
            selected_segments.append(np.asarray(seg[0], dtype=np.float32).reshape(3))
            selected_segments.append(np.asarray(seg[1], dtype=np.float32).reshape(3))
        if selected_segments:
            self.lines_selected.set_points(np.asarray(selected_segments, dtype=np.float32))
        else:
            self.lines_selected.set_points(np.zeros((0, 3), dtype=np.float32))

        if prev_selected != self.selected_constraint_ids:
            self._update_remove_button_label()

    def _update_sampled_clouds(self) -> None:
        if not self.sampled_cloud_visible or not self.sampled_clouds:
            return
        for cloud, ts in self.sampled_clouds:
            node = self.po.get_node(ts)
            if node:
                cloud.set_pose(_as_f_pose(node.get_pose()))

    def _update_trajectory(self) -> None:
        if (self.sampling_mode == mapping.SamplingMode.KEY_FRAMES and
                self._keyframe_ts_base):
            poses_np = self._keyframe_pose_stack()
        else:
            poses = self.po.get_poses(self.sampling_mode)
            poses_np = np.asarray(poses, dtype=np.float64)
        if poses_np.ndim != 3 or poses_np.shape[0] == 0:
            return
        xyz = poses_np[:, :3, 3].astype(np.float32)
        if self.opt_cloud.size != xyz.shape[0]:
            self.viz.remove(self.opt_cloud)
            self.opt_cloud = self._viz_module.Cloud(xyz.shape[0])
            self.opt_cloud.set_point_size(4.0)
            self.opt_cloud.set_key(
                np.tile(np.array([1.0, 0.5, 0.0, 1.0], dtype=np.float32), (xyz.shape[0], 1))
            )
            self.viz.add(self.opt_cloud)
        self.opt_cloud.set_xyz(xyz)
        self.opt_line.set_points(self._make_line_segments(xyz))

        if not self._camera_fitted and xyz.shape[0] > 0:
            try:
                if self._raw_xyz.size > 0:
                    combo = np.vstack([self._raw_xyz, xyz])
                    self._fit_camera_to_points(combo)
                else:
                    self._fit_camera_to_points(xyz)
            except Exception:
                self._fit_camera_to_points(xyz)
            self._camera_fitted = True

    def _apply_raw_trajectory_alignment(self, pose: np.ndarray) -> None:
        if self._raw_alignment_applied:
            return
        raw_xyz = self._raw_xyz
        aligned_xyz = _transform_xyz(raw_xyz, pose)

        self._raw_xyz = aligned_xyz
        if aligned_xyz.size:
            self.raw_cloud.set_xyz(aligned_xyz)
            self.raw_line.set_points(self._make_line_segments(aligned_xyz))
        else:
            self.raw_line.set_points(np.zeros((0, 3), dtype=np.float32))
        self._raw_alignment_applied = True
        self._camera_fitted = False

    def _update_all(self) -> None:
        self._update_trajectory()
        self._update_node_orientations()
        self._update_constraints()
        self._update_sampled_clouds()

    def _teardown_callbacks(self) -> None:
        # Avoid callback reference cycles between Python objects and nanobind
        # wrappers when the viz window is closed.
        try:
            self.solver_step_callback = (lambda: None)
            self.po.set_solver_step_callback(self.solver_step_callback)
        except Exception:
            pass

        for cloud in self.constraint_pick_clouds.values():
            try:
                cloud.clear_on_select()
            except Exception:
                pass

    def step_callback(self) -> None:
        if getattr(self, "_solving_active", False):
            try:
                total = self._solving_total_steps
                if total > 0:
                    self._solving_done_steps = min(self._solving_done_steps + 1, total)
                else:
                    self._solving_done_steps += 1
                done = self._solving_done_steps
                if total <= 0:
                    progress = f"Solving {done} step{'s' if done != 1 else ''}"
                    if getattr(self, "_running_until_converged", False):
                        progress = f"{progress} (solve until convergence)"
                else:
                    suffix = "step" if total == 1 else "steps"
                    progress = f"Solving {done} / {total} {suffix}"
                if self._prev_cost is not None and self._prev_cost != 0:
                    self._set_status(f"{progress} | prev cost: {self._prev_cost:.9g}")
                else:
                    self._set_status(f"{progress} | prev cost: NA")
            except Exception:
                pass
        self._update_all()
        self.viz.update()

    def run(self) -> None:
        self.viz.update()
        try:
            self.viz.run()
        finally:
            self._teardown_callbacks()
