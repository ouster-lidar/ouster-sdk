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
