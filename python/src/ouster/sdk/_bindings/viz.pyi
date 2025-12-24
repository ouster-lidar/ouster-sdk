"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Type annotations for viz python bindings.
"""
from typing import Callable, overload, Tuple, List, ClassVar, Optional
import numpy as np
from ouster.sdk.core import SensorInfo
from ouster.sdk._bindings.client import Mesh as SimpleMesh, Zrb
calref_palette: np.ndarray
spezia_palette: np.ndarray
spezia_cal_ref_palette: np.ndarray
grey_palette: np.ndarray
grey_cal_ref_palette: np.ndarray
viridis_palette: np.ndarray
viridis_cal_ref_palette: np.ndarray
magma_palette: np.ndarray
magma_cal_ref_palette: np.ndarray

class MouseButton:
    MOUSE_BUTTON_1: ClassVar[MouseButton]
    MOUSE_BUTTON_2: ClassVar[MouseButton]
    MOUSE_BUTTON_3: ClassVar[MouseButton]
    MOUSE_BUTTON_4: ClassVar[MouseButton]
    MOUSE_BUTTON_5: ClassVar[MouseButton]
    MOUSE_BUTTON_6: ClassVar[MouseButton]
    MOUSE_BUTTON_7: ClassVar[MouseButton]
    MOUSE_BUTTON_8: ClassVar[MouseButton]
    MOUSE_BUTTON_LAST: ClassVar[MouseButton]
    MOUSE_BUTTON_LEFT: ClassVar[MouseButton]
    MOUSE_BUTTON_RIGHT: ClassVar[MouseButton]
    MOUSE_BUTTON_MIDDLE: ClassVar[MouseButton]

    @property
    def name(self) -> str:
        """name(self: object) -> str
"""
        ...

    @property
    def value(self) -> int:
        ...
MOUSE_BUTTON_1: MouseButton
MOUSE_BUTTON_2: MouseButton
MOUSE_BUTTON_3: MouseButton
MOUSE_BUTTON_4: MouseButton
MOUSE_BUTTON_5: MouseButton
MOUSE_BUTTON_6: MouseButton
MOUSE_BUTTON_7: MouseButton
MOUSE_BUTTON_8: MouseButton
MOUSE_BUTTON_LAST: MouseButton
MOUSE_BUTTON_LEFT: MouseButton
MOUSE_BUTTON_RIGHT: MouseButton
MOUSE_BUTTON_MIDDLE: MouseButton

class MouseButtonEvent:
    MOUSE_BUTTON_RELEASED: ClassVar[MouseButtonEvent]
    MOUSE_BUTTON_PRESSED: ClassVar[MouseButtonEvent]

    @property
    def name(self) -> str:
        """name(self: object) -> str
"""
        ...

    @property
    def value(self) -> int:
        ...
MOUSE_BUTTON_RELEASED: MouseButtonEvent
MOUSE_BUTTON_PRESSED: MouseButtonEvent

class EventModifierKeys:
    MOD_SHIFT: ClassVar[EventModifierKeys]
    MOD_CONTROL: ClassVar[EventModifierKeys]
    MOD_ALT: ClassVar[EventModifierKeys]
    MOD_SUPER: ClassVar[EventModifierKeys]
    MOD_CAPS_LOCK: ClassVar[EventModifierKeys]
    MOD_NUM_LOCK: ClassVar[EventModifierKeys]
    MOD_NONE: ClassVar[EventModifierKeys]

    @property
    def name(self) -> str:
        """name(self: object) -> str
"""
        ...

    @property
    def value(self) -> int:
        ...
MOD_SHIFT: EventModifierKeys
MOD_CONTROL: EventModifierKeys
MOD_ALT: EventModifierKeys
MOD_SUPER: EventModifierKeys
MOD_CAPS_LOCK: EventModifierKeys
MOD_NUM_LOCK: EventModifierKeys
MOD_NONE: EventModifierKeys

class WindowCtx:

    @property
    def lbutton_down(self) -> bool:
        """True if the left mouse button is held"""
        ...

    @property
    def mbutton_down(self) -> bool:
        """True if the middle mouse button is held"""
        ...

    @property
    def mouse_x(self) -> float:
        """Current mouse x position"""
        ...

    @property
    def mouse_y(self) -> float:
        """Current mouse y position"""
        ...

    @property
    def viewport_width(self) -> int:
        """Current viewport width in pixels"""
        ...

    @property
    def viewport_height(self) -> int:
        """Current viewport height in pixels"""
        ...

    @property
    def window_width(self) -> int:
        """Current window width in screen coordinates"""
        ...

    @property
    def window_height(self) -> int:
        """Current window height in screen coordinates"""
        ...

    def normalized_coordinates(self, x: float, y: float) -> Tuple[float, float]:
        """Return 2d normalized viewport coordinates given window coordinates."""
        ...

    def aspect_ratio(self) -> float:
        """Return the aspect ratio of the viewport."""
        ...

class Camera:

    def reset(self) -> None:
        """Reset the camera view and fov."""
        ...

    def yaw(self, degrees: float) -> None:
        """Orbit the camera left or right about the camera target."""
        ...

    def set_yaw(self, degrees: float) -> None:
        """Set yaw in degrees."""
        ...

    def get_yaw(self) -> float:
        """Get yaw in degrees."""
        ...

    def roll(self, degrees: float) -> None:
        """Roll the camera left or right about the camera target."""
        ...

    def set_roll(self, degrees: float) -> None:
        """Set roll in degrees."""
        ...

    def get_roll(self) -> float:
        """Get roll in degrees."""
        ...

    def pitch(self, degrees: float) -> None:
        """Pitch the camera up or down."""
        ...

    def set_pitch(self, degrees: float) -> None:
        """Set pitch in degrees."""
        ...

    def get_pitch(self) -> float:
        """Get pitch in degrees."""
        ...

    def dolly(self, amount: int) -> None:
        """Move the camera towards or away from the target."""
        ...

    def set_dolly(self, log_distance: float) -> None:
        """Set the dolly (i.e. log distance) of the camera from the target."""
        ...

    def get_dolly(self) -> float:
        """Get the dolly (i.e. log distance) of the camera from the target."""
        ...

    def dolly_xy(self, x: float, y: float) -> None:
        """
             Move the camera in the XY plane of the camera view.

             Args:
                 x: horizontal offset
                 y: vertical offset
             """
        ...

    def set_view_offset(self, view_offset: np.ndarray) -> None:
        """Set view offset of a camera"""
        ...

    def get_view_offset(self) -> np.ndarray:
        """Get view offset of a camera"""
        ...

    def set_fov(self, degrees: float) -> None:
        """Set the diagonal field of view."""
        ...

    def get_fov(self) -> float:
        """Get the diagonal field of view in degrees."""
        ...

    def set_orthographic(self, state: bool) -> None:
        """Use an orthographic or perspective projection."""
        ...

    def is_orthographic(self) -> bool:
        """Get the orthographic state."""
        ...

    def set_proj_offset(self, x: float, y: float) -> None:
        """
             Set the 2d position of camera target in the viewport.

             Args:
                 x: horizontal position in in normalized coordinates [-1, 1]
                 y: vertical position in in normalized coordinates [-1, 1]
             """
        ...

    def get_proj_offset(self) -> np.ndarray:
        """Get the 2d position of a camera target in the viewport."""
        ...

    def set_target(self, pose: np.ndarray) -> None:
        """
                 Set the camera target pose (inverted pose).

                 Args:
                    pose: 4x4 column-major homogeneous transformation matrix
             """
        ...

    def get_target(self) -> np.ndarray:
        """Get a pose of the camera target."""
        ...

class TargetDisplay:

    def enable_rings(self, state: bool) -> bool:
        """Enable or disable distance ring display."""
        ...

    def set_ring_size(self, n: int) -> None:
        """Set the distance between rings."""
        ...

    def set_ring_line_width(self, line_width: int) -> None:
        """Set the line width of the rings."""
        ...

class Cloud:

    @overload
    def __init__(self, n_points: int) -> None:
        """
                 ``def __init__(self, n_points: int, extrinsics: np.ndarray) -> None:``

                 Unstructured point cloud for visualization.

                 Call set_xyz() to update

                 Args:
                    num_points: number of points
                    extrinsics: sensor extrinsic calibration. 4x4 column-major
                                homogeneous transformation matrix.
             

                 ``def __init__(self, si: SensorInfo) -> None:``

                 Structured point cloud for visualization.

                 Call set_range() to update

                 Args:
                    info: sensor metadata
             """
        ...

    @overload
    def __init__(self, si: SensorInfo) -> None:
        """
                 ``def __init__(self, n_points: int, extrinsics: np.ndarray) -> None:``

                 Unstructured point cloud for visualization.

                 Call set_xyz() to update

                 Args:
                    num_points: number of points
                    extrinsics: sensor extrinsic calibration. 4x4 column-major
                                homogeneous transformation matrix.
             

                 ``def __init__(self, si: SensorInfo) -> None:``

                 Structured point cloud for visualization.

                 Call set_range() to update

                 Args:
                    info: sensor metadata
             """
        ...

    def set_range(self, range: np.ndarray) -> None:
        """
                Set the range values.

                Args:
                  range: array of at least as many elements as there are points,
                         representing the range of the points
              """
        ...

    def set_key(self, key: np.ndarray) -> None:
        """
                 Set the key values, used for colouring.

                 Number of elements defines the type of Cloud coloration:
                 - num elements == cloud.get_size(): MONO with palette
                 - 3 dimensions with the last dimesion: 3 - RGB, 4 - RGBA,
                   no palette used

                 Args:
                    key: array of at least as many elements as there are
                         points, preferably normalized between 0 and 1
             """
        ...

    def set_key_rgb(self, key: np.ndarray) -> None:
        """
                 Set the key to RGB values, used for colouring.

                 Size must be:
                 - 2 dimensions with last dimension 3 for unordered pointclouds
                 - 3 dimensions with last dimension 3 for ordered pointclouds

                 Args:
                    key: array of RGB colors as many elements as there are
                         points
             """
        ...

    def set_key_rgba(self, key: np.ndarray) -> None:
        """
                 Set the key to RGBA values, used for colouring.

                 Size must be:
                 - 2 dimensions with last dimension 4 for unordered pointclouds
                 - 3 dimensions with last dimension 4 for ordered pointclouds

                 Args:
                    key: array of RGBA colors as many elements as there are
                         points
             """
        ...

    def set_mask(self, mask: np.ndarray) -> None:
        """
                 Set the RGBA mask values, used as an overlay on top of the key.

                 Args:
                    mask: array of at least 4x as many elements as there
                          are points, preferably normalized between 0 and 1
             """
        ...

    def set_xyz(self, xyz: np.ndarray) -> None:
        """
                 Set the XYZ values.

                 :param xyz:  Supports 3 formats:
                              * array of exactly 3n where n is the number of
                                points, so that the xyz position of the ith
                                point is ``i``, ``i+n``, ``i+2n``.
                              * array of (N, 3) where N is the number of points
                              * array of (H, W, 3) where H*W is the number of
                                points
                 :type xyz: array of np.float32. Pybind should also cast other
                            numpy types (but we haven't tested thoroughly)
             """
        ...

    def set_pose(self, pose: np.ndarray) -> None:
        """
                 Set the ith point cloud pose.

                 Args:
                    pose: 4x4 column-major homogeneous transformation matrix
             """
        ...

    def set_point_size(self, size: float) -> None:
        """
            Set point size.

            Args:
                size: point size
        """
        ...

    def set_palette(self, palette: np.ndarray) -> None:
        """
            Set the point cloud color palette.

            Args:
                palette: the new palette to use, must have size 3*palette_size
        """
        ...

    def set_column_poses(self, column_poses: np.ndarray) -> None:
        """
                 Set scan poses (per every column).

                 Args:
                    column_poses: array of poses (Wx4x4) per every column.
             """
        ...

    @property
    def size(self) -> int:
        """Number of points in a cloud"""
        ...

    @property
    def cols(self) -> int:
        """Number of columns in a cloud (1 if point cloud is unstructured"""
        ...

class Image:

    def __init__(self) -> None:
        """__init__(self: ouster.sdk._bindings.viz.Image) -> None
"""
        ...

    def set_image(self, image: np.ndarray) -> None:
        """
                 Set the image data, MONO or RGB/RGBA depending on dimensions.

                 Color palette is applied for MONO mode if set_palette() was
                 used to set the palette, otherwise MONO mode makes the
                 monochrome image.

                 Args:
                    image: 2D array of floats for a monochrome image or 3D array
                           with RGB or RGBA components for color image.
             """
        ...

    def set_mask(self, image: np.ndarray) -> None:
        """
                 Set the RGBA mask.

                 Args:
                    mask: M x N x 4 array with RGBA mask
             """
        ...

    def set_position(self, x0: float, x1: float, y0: float, y1: float) -> None:
        """
            Set the display position of the image.

            Coordinates are {x_min, x_max, y_max, y_min} in sort-of normalized
            screen coordinates: y is in [-1, 1], and x uses the same scale
            (i.e. window width is ignored). This is currently just the same
            method the previous hard-coded 'image_frac' logic was using; I
            believe it was done this way to allow scaling with the window
            while maintaining the aspect ratio.

            Args:
                x_min: min x
                x_max: max x
                y_min: min y
                y_max: max y
        """
        ...

    def set_hshift(self, hshift: float) -> None:
        """
            Set horizontal shift in normalized viewport screen width coordinate.

            This may be used to "snap" images to the left/right screen edges.

            Some example values:
              ``0`` - default, image is centered horizontally on the screen
              ``-0.5`` - image moved to the left for the 1/4 of a horizontal viewport
              ``-1`` - image moved to the left for the 1/2 of a horizontal viewport
              ``+1`` - image moved to the right for the 1/2 of a horizontal viewport
              ``+0.5`` - image moved to the right for the 1/4 of a horizontal viewport
        """
        ...

    def set_palette(self, palette: np.ndarray) -> None:
        """
            Set the image color palette.

            Args:
                palette: the new palette to use, must have size 3*palette_size
        """
        ...

    def clear_palette(self) -> None:
        """Removes the image palette and use keys as grey color in MONO"""
        ...

    def viewport_coordinates_to_image_pixel(self, ctx: WindowCtx, x: float, y: float) -> Tuple[int, int]:
        """Returns the image pixel as a (row, col) tuple given window coordinates,
             or None if the given window coordinate is not within the image."""
        ...

    def image_pixel_to_viewport_coordinates(self, ctx: WindowCtx, pixel: Tuple[int, int]) -> Tuple[float, float]:
        """Returns the window pixel (x, y) given an image (row, col) pixel."""
        ...

    def pixel_size(self) -> Tuple[float, float]:
        """Returns the pixel size (w, h) in window pixels."""
        ...

class Vec3f:

    def __init__(self, x: float, y: float, z: float) -> None:
        """__init__(self: ouster.sdk._bindings.viz.Vec3f, arg0: float, arg1: float, arg2: float) -> None
"""
        ...

class Vertex:

    def __init__(self, position: Vec3f, normal: Vec3f) -> None:
        """__init__(self: ouster.sdk._bindings.viz.Vertex, arg0: numpy.ndarray[numpy.float32[3, 1]], arg1: numpy.ndarray[numpy.float32[3, 1]]) -> None
"""
        ...

    @property
    def position(self) -> Vec3f:
        """vertex position"""
        ...

    @property
    def normal(self) -> Vec3f:
        """vertex normal"""
        ...

class Mesh:

    def __init__(self, vertices: List[Vertex], edge_indices: np.ndarray, face_vertices: np.ndarray, face_rgba: Tuple[float, float, float, float], edge_rgba: Tuple[float, float, float, float]) -> None:
        """Creates a mesh."""
        ...

    @staticmethod
    def from_simple_mesh(simple_mesh: SimpleMesh) -> 'Mesh':
        """from_simple_mesh(arg0: ouster.sdk._bindings.client.Mesh) -> ouster.sdk._bindings.viz.Mesh
"""
        ...

    def set_edge_rgba(self, rgba: Tuple[float, float, float, float]) -> None:
        """
        Set the edge color of the mesh.

        Args:
            rgba: 4 value tuple of RGBA color
    """
        ...

    def set_transform(self, pose: np.ndarray) -> None:
        """
             Set the transform defining the mesh.

             Args:
                pose: 4x4 pose matrix
         """
        ...

class Cuboid:

    def __init__(self, pose: np.ndarray, rgba: Tuple[float, ...]) -> None:
        """
                 Creates cuboid.

                 Args:
                    pose: 4x4 pose matrix
                    rgba: 4 value tuple of RGBA color
             """
        ...

    def set_transform(self, pose: np.ndarray) -> None:
        """
                 Set the transform defining the cuboid.

                 Applied to a unit cube centered at the origin.

                 Args:
                    pose: 4x4 pose matrix
             """
        ...

    def set_rgba(self, rgba: Tuple[float, ...]) -> None:
        """
            Set the color of the cuboid.

            Args:
                rgba: 4 value tuple of RGBA color
        """
        ...

class Lines:

    def __init__(self, pose: np.ndarray, rgba: Tuple[float, ...]) -> None:
        """
                 Creates lines.

                 Args:
                    pose: 4x4 pose matrix
                    rgba: 4 value tuple of RGBA color
             """
        ...

    def set_transform(self, pose: np.ndarray) -> None:
        """
                 Set the transform for the lines.

                 Args:
                    pose: 4x4 pose matrix
             """
        ...

    def set_points(self, points: np.ndarray) -> None:
        """
                 Set the line points.

                 Args:
                    points: array of floats
             """
        ...

    def set_rgba(self, rgba: Tuple[float, ...]) -> None:
        """
            Set the color of the lines.

            Args:
                rgba: 4 value tuple of RGBA color
        """
        ...

class Label:

    @overload
    def __init__(self, text: str, x: float, y: float, z: float) -> None:
        """
                 ``def __init__(self, text: str, x: float, y: float, z: float) -> None:``

                 Creates 3D Label.

                 Args:
                    text: label text
                    x,y,z: label location
             

                 ``def __init__(self, text: str, x: float, y: float, align_right: bool = ..., align_top: bool = ...) -> None:``

                 Creates 2D Label.

                 Args:
                    text: label text
                    x,y: label 2D location in screen coords ``[0..1]``, corresponding to top left corner of label
                    align_right: if ``True`` - anchor point of the label is the right side
                    align_top: if ``True`` - anchor point of the label is the top side
             """
        ...

    @overload
    def __init__(self, text: str, x: float, y: float, align_right: bool=..., align_top: bool=...) -> None:
        """
                 ``def __init__(self, text: str, x: float, y: float, z: float) -> None:``

                 Creates 3D Label.

                 Args:
                    text: label text
                    x,y,z: label location
             

                 ``def __init__(self, text: str, x: float, y: float, align_right: bool = ..., align_top: bool = ...) -> None:``

                 Creates 2D Label.

                 Args:
                    text: label text
                    x,y: label 2D location in screen coords ``[0..1]``, corresponding to top left corner of label
                    align_right: if ``True`` - anchor point of the label is the right side
                    align_top: if ``True`` - anchor point of the label is the top side
             """
        ...

    def set_text(self, text: str) -> None:
        """
            Update label text.

            Args:
                text: new text to display
        """
        ...

    @overload
    def set_position(self, x: float, y: float, z: float) -> None:
        """
                ``def set_position(self, x: float, y: float, z: float) -> None:``

                 Set label position. Position correspnods to top left (viewer's left) of label.

                 Args:
                    x,y,z: label position in 3D
            

                 ``def set_position(self, x: float, y: float, align_right: bool = ...) -> None:``

                 Set position of the 2D label.

                 Args:
                    x,y: label 2D position in screen coords ``[0..1]``
                    align_right: if ``True`` - anchor point of the label is the right side
                    align_top: if ``True`` - anchor point of the label is the top side
             """
        ...

    @overload
    def set_position(self, x: float, y: float, align_right: bool=..., align_top: bool=...) -> None:
        """
                ``def set_position(self, x: float, y: float, z: float) -> None:``

                 Set label position. Position correspnods to top left (viewer's left) of label.

                 Args:
                    x,y,z: label position in 3D
            

                 ``def set_position(self, x: float, y: float, align_right: bool = ...) -> None:``

                 Set position of the 2D label.

                 Args:
                    x,y: label 2D position in screen coords ``[0..1]``
                    align_right: if ``True`` - anchor point of the label is the right side
                    align_top: if ``True`` - anchor point of the label is the top side
             """
        ...

    def set_scale(self, scale: float) -> None:
        """
             Set scaling factor of the label.

             Args:
                scale: text scale factor
         """
        ...

    def set_rgba(self, rgba: Tuple[float, ...]) -> None:
        """
            Set the color of the label.

            Args:
                rgba: 4 value tuple of RGBA color
        """
        ...

    @property
    def text_height(self) -> float:
        """
                               Get the height of the label text.
                               """
        ...

class PointViz:

    def __init__(self, name: str, fix_aspect: bool=..., window_width: int=..., window_height: int=..., maximized: bool=..., fullscreen: bool=..., borderless: bool=...) -> None:
        """__init__(self: ouster.sdk._bindings.viz.PointViz, name: str, fix_aspect: bool = False, window_width: int = 800, window_height: int = 600, maximized: bool = False, fullscreen: bool = False, borderless: bool = False) -> None
"""
        ...

    def run(self) -> None:
        """
             Run the visualizer rendering loop.

             Must be called from the main thread. Will return when ``running(False)`` is
             called from another thread or when the visualizer window is closed.
        """
        ...

    def run_once(self) -> None:
        """Run one iteration of the main loop for rendering and input handling."""
        ...

    @overload
    def running(self) -> bool:
        """Check if the rendering loop is running.
Shut down the visualizer and break out of the rendering loop."""
        ...

    @overload
    def running(self, state: bool) -> None:
        """Check if the rendering loop is running.
Shut down the visualizer and break out of the rendering loop."""
        ...

    def update(self) -> None:
        """Show updated data in the next rendered frame."""
        ...

    def visible(self, state: bool) -> None:
        """Toggle if the PointViz window is visible"""
        ...

    def cursor_visible(self, state: bool) -> None:
        """Toggle if the cursor is visible when over this window"""
        ...

    def push_key_handler(self, f: Callable[[WindowCtx, int, int], bool]) -> None:
        """Add a callback for handling keyboard input."""
        ...

    def pop_key_handler(self) -> None:
        """Remove the last added callback for handling keyboard input."""
        ...

    def push_mouse_button_handler(self, f: Callable[[WindowCtx, MouseButton, MouseButtonEvent, EventModifierKeys], bool]) -> None:
        """Add a callback for handling mouse button events."""
        ...

    def pop_mouse_button_handler(self) -> None:
        """Remove the last added callback for mouse button events."""
        ...

    def push_scroll_handler(self, f: Callable[[WindowCtx, float, float], bool]) -> None:
        """Add a callback for handling mouse scroll events."""
        ...

    def pop_scroll_handler(self) -> None:
        """Remove the last added callback for mouse scroll events."""
        ...

    def push_mouse_pos_handler(self, f: Callable[[WindowCtx, float, float], bool]) -> None:
        """Add a callback for handling mouse position events."""
        ...

    def pop_mouse_pos_handler(self) -> None:
        """Remove the last added callback for mouse position events."""
        ...

    def push_frame_buffer_resize_handler(self, f: Callable[[WindowCtx], bool]) -> None:
        """Add a callback for handling window resize events."""
        ...

    def pop_frame_buffer_resize_handler(self) -> None:
        """Remove the last added callback for window resize events."""
        ...

    @overload
    def get_screenshot(self, scale_factor: float=...) -> np.ndarray:
        """Gets a screenshot with an explicit width and height. Returns the pixels.
Gets a screenshot using a scale factor, which is a multiplier over the window width and height. Returns the pixels."""
        ...

    @overload
    def get_screenshot(self, width: int, height: int) -> np.ndarray:
        """Gets a screenshot with an explicit width and height. Returns the pixels.
Gets a screenshot using a scale factor, which is a multiplier over the window width and height. Returns the pixels."""
        ...

    @overload
    def save_screenshot(self, path: str, scale_factor: float=...) -> str:
        """Saves a screenshot using a scale factor, which is a multiplier over the window width and height. Returns the resulting file name.
Saves a screenshot with an explicit width and height. Returns the resulting file name."""
        ...

    @overload
    def save_screenshot(self, path: str, width: int, height: int) -> str:
        """Saves a screenshot using a scale factor, which is a multiplier over the window width and height. Returns the resulting file name.
Saves a screenshot with an explicit width and height. Returns the resulting file name."""
        ...

    @overload
    def toggle_screen_recording(self, scale_factor: float=...) -> bool:
        """Toggle screen recording. Returns true if started, false if stopped
Toggle screen recording with explicit width and height. Returns true if started, false if stopped."""
        ...

    @overload
    def toggle_screen_recording(self, width: int, height: int) -> bool:
        """Toggle screen recording. Returns true if started, false if stopped
Toggle screen recording with explicit width and height. Returns true if started, false if stopped."""
        ...

    @property
    def camera(self) -> Camera:
        """Get a reference to the camera controls."""
        ...

    @property
    def target_display(self) -> TargetDisplay:
        """Get a reference to the target display."""
        ...

    @property
    def viewport_width(self) -> int:
        """Current viewport width in pixels"""
        ...

    @property
    def viewport_height(self) -> int:
        """Current viewport height in pixels"""
        ...

    @property
    def window_width(self) -> int:
        """Current window width in screen coordinates"""
        ...

    @property
    def window_height(self) -> int:
        """Current window height in screen coordinates"""
        ...

    @overload
    def add(self, cloud: Cloud) -> None:
        """
             Add an object to the scene.

             Args:
                 obj: A cloud, label, image or cuboid."""
        ...

    @overload
    def add(self, image: Image) -> None:
        """
             Add an object to the scene.

             Args:
                 obj: A cloud, label, image or cuboid."""
        ...

    @overload
    def add(self, cuboid: Cuboid) -> None:
        """
             Add an object to the scene.

             Args:
                 obj: A cloud, label, image or cuboid."""
        ...

    @overload
    def add(self, label: Label) -> None:
        """
             Add an object to the scene.

             Args:
                 obj: A cloud, label, image or cuboid."""
        ...

    @overload
    def add(self, lines: Lines) -> None:
        """
             Add an object to the scene.

             Args:
                 obj: A cloud, label, image or cuboid."""
        ...

    @overload
    def add(self, mesh: Mesh) -> None:
        """
             Add an object to the scene.

             Args:
                 obj: A cloud, label, image or cuboid."""
        ...

    @overload
    def remove(self, cloud: Cloud) -> bool:
        """
             Remove an object from the scene.

             Args:
                 obj: A cloud, label, image or cuboid.

             Returns:
                 True if the object was in the scene and was removed.
             """
        ...

    @overload
    def remove(self, image: Image) -> bool:
        """
             Remove an object from the scene.

             Args:
                 obj: A cloud, label, image or cuboid.

             Returns:
                 True if the object was in the scene and was removed.
             """
        ...

    @overload
    def remove(self, cuboid: Cuboid) -> bool:
        """
             Remove an object from the scene.

             Args:
                 obj: A cloud, label, image or cuboid.

             Returns:
                 True if the object was in the scene and was removed.
             """
        ...

    @overload
    def remove(self, label: Label) -> bool:
        """
             Remove an object from the scene.

             Args:
                 obj: A cloud, label, image or cuboid.

             Returns:
                 True if the object was in the scene and was removed.
             """
        ...

    @overload
    def remove(self, lines: Lines) -> bool:
        """
             Remove an object from the scene.

             Args:
                 obj: A cloud, label, image or cuboid.

             Returns:
                 True if the object was in the scene and was removed.
             """
        ...

    @overload
    def remove(self, mesh: Mesh) -> bool:
        """
             Remove an object from the scene.

             Args:
                 obj: A cloud, label, image or cuboid.

             Returns:
                 True if the object was in the scene and was removed.
             """
        ...

    @property
    def fps(self) -> float:
        """Frames per second, updated every second in the draw() func"""
        ...

    def set_background_color(self, rgba: Tuple[float, ...]) -> None:
        """
            Set the background color of the viz.

            Args:
                rgba: 4 value tuple of RGBA color
        """
        ...

    def set_notification(self, text: str, duration: float=2.0) -> None:
        """
             Set a notification text to be displayed in the top-right corner of
             the window.

             Args:
                 text: Text to display. Empty string to clear.
                 duration: Duration in seconds to display the text (default: 2.0 seconds).
                           0 means indefinitely.
         """
        ...

    def notification_active(self) -> bool:
        """Unbound function"""
        ...

    @property
    def notifications_enabled(self) -> bool:
        """Enable or disable notifications."""
        ...

    @notifications_enabled.setter
    def notifications_enabled(self, state: bool) -> None:
        """Enable or disable notifications."""
        ...

def add_default_controls(viz: PointViz) -> None:
    """Add default keyboard and mouse bindings to a visualizer instance."""
    ...

def precompute_voxel_vertices(voxel_size: float) -> np.ndarray:
    """
              Pre-computes direction and offset vectors for every vertex in the pixel grid.

              This is an expensive, one-time calculation that generates a lookup table
              which can be reused for creating meshes from any number of image pairs of
              the same resolution from the same sensor.

              Args:
                  metadata (ouster.sdk.sensor.SensorInfo): The sensor metadata.

              Returns:
                  list[list[VoxelVertexData]]: A 2D lookup table of geometric data
                  for the grid corners.
    """
    ...

def voxel_style_mesh_from_zone_image_pair(zrb: Zrb, sensor_info: SensorInfo, voxel_vertices: np.ndarray) -> Mesh:
    """voxel_style_mesh_from_zone_image_pair(zone_image_pair: ouster.sdk._bindings.client.Zrb, voxel_vertex_data: ouster.sdk._bindings.client.SensorInfo, sensor_info: list[list[ouster::sdk::viz::VoxelVertexData]]) -> ouster.sdk._bindings.viz.Mesh
"""
    ...