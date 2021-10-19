"""Type annotations for sensor tools python bindings."""

from ouster.client import LidarScan, SensorInfo


class PyViz():
    def __init__(self, si: SensorInfo):
        ...

    def draw(self,
             ls: LidarScan,
             cloud_ind: int = ...,
             cloud_swap: bool = ...,
             show_image: bool = ...) -> None:
        ...

    def loop(self) -> None:
        ...

    def quit(self) -> None:
        ...

    def is_quit(self) -> bool:
        ...
