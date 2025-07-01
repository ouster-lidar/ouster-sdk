"""
This example demonstrates how to handle mouse events in images.
"""
import numpy as np
from random import randrange
from ouster.sdk.viz import (
    PointViz, add_default_controls, Image, calref_palette,
    WindowCtx, MouseButton, MouseButtonEvent, EventModifierKeys, Label)
from typing import Tuple, Optional

viz = PointViz('Image Events')
img = Image()
img_size = (randrange(4, 7), randrange(7, 11))

img_data = 0.5 * np.random.rand(*img_size)
img.set_image(img_data)
img_pos = (-0.85, 0.0, -0.35, 0.75)
img.set_position(*img_pos)
img.set_palette(calref_palette)
hshift = -0.333
img.set_hshift(hshift)
label = Label("hello", 0.0, 0.0)
window_size_label = Label("", 0.05, 0.95)
window_size_label.set_scale(2)
viz.add(label)
viz.add(window_size_label)
hello_pixel: Optional[Tuple[int, int]] = None


def update_window_size_label(position: Tuple[float, float], text: str):
    global window_size_label
    window_size_label.set_text(text)
    window_size_label.set_position(*position)


def draw_pixels(ctx: WindowCtx, x: float, y: float):
    global hello_pixel
    if ctx.lbutton_down:
        hello_pixel = img.viewport_coordinates_to_image_pixel(ctx, x, y)
        # if the pixel location is a valid location in the image data
        if hello_pixel[0] >= 0 and hello_pixel[0] < img_size[0] and \
            hello_pixel[1] >= 0 and hello_pixel[1] < img_size[1]:

            img_data[hello_pixel] = 1.0
            img.set_image(img_data)
            pixel_center = img.image_pixel_to_viewport_coordinates(ctx, hello_pixel)
            label.set_position(
                pixel_center[0] / ctx.viewport_width,
                pixel_center[1] / ctx.viewport_height
            )
            viz.update()


def mouse_pos_handler(ctx: WindowCtx, x: float, y: float) -> bool:
    draw_pixels(ctx, x, y)
    return False


def mouse_button_handler(
    ctx: WindowCtx,
    button: MouseButton,
    event: MouseButtonEvent,
    mods: EventModifierKeys
) -> bool:
    draw_pixels(ctx, ctx.mouse_x, ctx.mouse_y)
    return False


def resize_handler(ctx: WindowCtx) -> bool:
    # updates the position of the window size text to be in the lower-left
    # PointViz uses a coordinate system with (0, 0) representing the lower-left
    # and (1, 1) representing the upper-right for Label instances.
    x = 10
    y = ctx.viewport_height - 10
    world_x = 1.0 / ctx.viewport_height * x / ctx.aspect_ratio()
    world_y = 1.0 * (y / ctx.viewport_height)
    pos = (world_x, world_y)
    update_window_size_label(pos, f"{ctx.viewport_width}x{ctx.viewport_height}")

    # update "hello"
    if hello_pixel:
        pixel_center = img.image_pixel_to_viewport_coordinates(ctx, hello_pixel)
        label.set_position(
            pixel_center[0] / ctx.viewport_width,
            pixel_center[1] / ctx.viewport_height
        )
    viz.update()
    return True


viz.add(img)
add_default_controls(viz)
viz.push_mouse_button_handler(mouse_button_handler)
viz.push_mouse_pos_handler(mouse_pos_handler)
viz.push_frame_buffer_resize_handler(resize_handler)
viz.update()
viz.run()
