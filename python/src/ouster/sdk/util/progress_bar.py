from typing import Optional
import time


def progressbar(progress, total, prefix="", suffix=""):
    """
    Displays progress in the console as a percentage.

    Args:
        progress: The current progress (number of items completed).
        total: The total number of items.
        prefix: A prefix string to display before the progress bar (optional).
        suffix: A suffix string to display after the progress bar (optional).
    """
    if total == 0:
        raise ValueError(
            "Progress cannot be displayed for a total of 0 items.")
    progress = total if progress > total else progress
    percent = round(100 * progress / total, 1)
    filled_length = int(round(percent * 20 / 100))
    bar = f'[{filled_length * "#"}{(20 - filled_length) * "-"}]'
    print(f'{prefix} {bar} {percent}% {suffix}', end="\r")


class ProgressBar:
    _last_time: Optional[float] = None
    _total: int

    def __init__(self, total, alpha = 0.05, unit=""):
        self._total = total
        self._iteration_time = 0
        self._alpha = alpha
        if len(unit):
            self._unit = f" {unit}/sec"
        else:
            self._unit = "/sec"

    def clear(self):
        # \r = Go to the start of the line
        # \033[2K = Clear the entire line
        print("\r\033[2K", end="")

    def update(self, progress, prefix="", suffix=""):
        now = time.monotonic()
        if self._last_time is not None:
            dt = now - self._last_time
            if self._iteration_time == 0:
                self._iteration_time = dt
            else:
                self._iteration_time = self._iteration_time * (1.0 - self._alpha) + dt * self._alpha
            # make sure we dont divide by zero or have negative
            if self._iteration_time <= 0:
                self._iteration_time = 0.00001
            scan_rate = 1.0 / self._iteration_time
            rate = f"{scan_rate:>5.0f}"
            remaining = max(0, self._total - progress)
            if self._total == 0:
                eta = ""
            else:
                eta = f"{remaining / scan_rate:>4.0f} sec remaining"
        else:
            rate = "?"
            eta = ""
        self._last_time = now

        if self._total == 0:
            total = 100
            progress = progress % (total)
            filled_length = progress // 5
            bar = f'[{filled_length * "-"}#{(19 - filled_length) * "-"}]'
            print(f'{prefix} {bar} {suffix} {rate}{self._unit} {eta}', end="\r")
        else:
            total = self._total
            progress = total if progress > total else progress
            percent = round(100 * progress / total, 1)
            filled_length = int(round(percent * 20 / 100))
            bar = f'[{filled_length * "#"}{(20 - filled_length) * "-"}]'
            print(f'{prefix} {bar} {percent:>5}% {suffix} {rate}{self._unit} {eta}', end="\r")

    def __enter__(self):
        """Called when entering the 'with' block."""
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """
        Called when exiting the 'with' block.
        """
        # Clear the progress bar line
        self.clear()

        return None
