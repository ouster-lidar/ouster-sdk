#  type: ignore
"""Miscellaneous utilites.

- extract beam indices to utility library:
  + use for tool to visualize 128-beam data as other prod lines
  + use for internal tool to convert 128-beam pcaps to other prod lines
"""

from io import FileIO
import hashlib
import click


click_ro_file = click.Path(exists=True, dir_okay=False, readable=True)


def md5file(path: str) -> str:
    """Calculate md5sum of a file."""
    bufsize = 256 * 1024
    md5 = hashlib.md5()
    buf = memoryview(bytearray(bufsize))
    with FileIO(path, 'rb') as f:
        for n in iter(lambda: f.readinto(buf), 0):
            md5.update(buf[:n])
    return md5.hexdigest()


ROS_MODULES_ERROR_MSG = """
Error: {err_msg}

Please verify that ROS Python modules are available.

The best option is to try to install unofficial rospy packages that work
with python3.7,3.8 on Ubuntu 18.04/20.04 and Debian 10 without ROS:

    pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf2_ros

NOTE: If during the attempt to run the above command you get an error:

    EnvironmentError: 404 Client Error: Not Found for url: https://pypi.org/simple/rospy/

Please check installed `pip` version (20.0+ works well with extra indexes), and
if needed upgrade `pip` with (in a sourced venv):

    pip install pip -U

Some users have even more packages missing so they may need to install additionally:

    pip install PyYAML pycryptodome pycryptodomex

"""


def import_rosbag_modules(raise_on_fail: bool = False):
    try:
        import rosbag  # noqa: F401
        import rospy   # noqa: F401
        import genpy   # noqa: F401
    except ImportError as err:
        if raise_on_fail:
            raise ModuleNotFoundError(
                ROS_MODULES_ERROR_MSG.format(err_msg=str(err)))
        return False
    return True
