from typing import Any, Tuple, Union
import numpy as np
from ouster.sdk.viz import PointViz, Image, Label, Mesh, Cloud


class Toggle:
    def __init__(self, viz: PointViz, item: Any, initially_visible: bool = False):
        self._viz = viz
        self._item = item
        self._visible = False
        self.visible = initially_visible

    def hide(self):
        self.visible = False

    def show(self):
        self.visible = True

    def toggle(self):
        self.visible = not self.visible

    @property
    def visible(self) -> bool:
        return self._visible

    @visible.setter
    def visible(self, visible: bool):
        """Depending on the visible parameter,
        add or remove the image from the PointViz
        depending on whether it's already added or removed."""
        if visible and not self._visible:
            self._viz.add(self._item)
        if not visible and self._visible:
            self._viz.remove(self._item)
        self._visible = visible


class ToggleImage(Toggle):
    """
    This class encapsulates properties of an image, such as whether it is visible in the supplied PointViz instance.
    This makes up for a deficiency in PointViz which requires that API users remember whether
    a widget has been added or not.
    """
    def __init__(self, viz: PointViz, initially_visible: bool = False):
        self._image = Image()
        super().__init__(viz, self._image, initially_visible)
        self._hshift = 0.0
        self._position = (0.0, 0.0, 0.0, 0.0)

    def set_image(self, image: np.ndarray):
        self._image.set_image(image)

    @property
    def hshift(self):
        return self._hshift

    @hshift.setter
    def hshift(self, hshift: float):
        self._hshift = hshift
        self._image.set_hshift(hshift)

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, position: Tuple[float, float, float, float]):
        self._position = position
        self._image.set_position(*self._position)


class ToggleLabel(Toggle):
    """
    This class encapsulates properties of a label, such as whether it is visible in the supplied PointViz instance.
    This makes up for a deficiency in PointViz which requires that API users remember whether
    a widget has been added or not.
    """
    def __init__(
        self, viz: PointViz,
        initial_text: str,
        initial_position: Union[Tuple[float, float], Tuple[float, float, float]],
        initially_visible: bool = False,
        align_right: bool = False,
        align_top: bool = False,
    ):
        self._position = initial_position
        self._scale = 1.0
        self._text = initial_text
        self._rgba = (1, 1, 1, 1)
        if len(initial_position) == 2:
            self._label = Label(initial_text, initial_position[0], initial_position[1], align_right, align_top)
        elif len(initial_position) == 3:
            self._label = Label(initial_text, *initial_position)
        super().__init__(viz, self._label, initially_visible)

    @property
    def text(self):
        return self._text

    @text.setter
    def text(self, text: str):
        self._label.set_text(text)

    @property
    def scale(self):
        return self._scale

    @scale.setter
    def scale(self, scale: float):
        self._scale = scale
        self._label.set_scale(scale)

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, position: Union[Tuple[float, float], Tuple[float, float, float]]):
        self._position = position
        self._label.set_position(*self._position)

    @property
    def text_height(self):
        return self._label.text_height

    @property
    def rgba(self):
        return self._rgba

    @rgba.setter
    def rgba(self, rgba: Tuple[float, float, float, float]):
        self._label.set_rgba(rgba)


class ToggleMesh(Toggle):
    def __init__(self, viz: PointViz, mesh: Mesh, initially_visible: bool = False):
        super().__init__(viz, mesh, initially_visible)


class ToggleCloud(Toggle):
    def __init__(self, viz: PointViz, cloud: Cloud, initially_visible: bool = False):
        super().__init__(viz, cloud, initially_visible)
