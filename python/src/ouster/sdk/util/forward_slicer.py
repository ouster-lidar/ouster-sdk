from typing import Iterator


class ForwardSlicer:
    """ForwardSlicer provides slicing methods to slice up a container with step
    to containers that only support forward slicing"""

    @staticmethod
    def normalize(key: slice, L: int) -> slice:

        def _slice_step(step):
            if step is None:
                return 1
            if step == 0:
                raise ValueError("slice step cannot be zero")
            return step

        def _slice_clamp(value, length, default):
            if value is None:
                return default
            if value < 0:
                return max(0, length + value)
            return min(value, length)

        step = _slice_step(key.step)
        if step > 0:
            start = _slice_clamp(key.start, L, 0)
            stop = _slice_clamp(key.stop, L, L)
        else:
            start = _slice_clamp(key.stop, L, -1) + 1
            stop = min(L, _slice_clamp(key.start, L, L) + 1)

        return slice(start, stop, step)

    @staticmethod
    def slice(data_iter: Iterator, key: slice):
        """
        Performs forward slicing on a dataset with step

        Parameters:
        - key: must be a normalized slice key with relation to the used data_iter.
            a normalized slice key is one where key.start < key.stop and no non-values
        """

        def _stepper(data_iter, start, stop, step):
            out = []
            if step < 0:
                # align with the end
                step = -step
                aligned_start = (stop - 1) - (stop - start) // step * step
                if aligned_start < start:
                    aligned_start += step
                for _ in range(aligned_start - start):
                    next(data_iter)
            while True:
                try:
                    out.append(next(data_iter))
                    for _ in range(step - 1):
                        next(data_iter)
                except StopIteration:
                    break
            return out

        return _stepper(data_iter, key.start, key.stop, key.step)
