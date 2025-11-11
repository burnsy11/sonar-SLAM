import time
import timeit
from functools import wraps
import numpy as np
from tqdm.auto import tqdm
from threading import Event
import rclpy
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration as RclpyDuration

offline = False
callback_lock_event = Event()
callback_lock_event.set()

# Global logger reference for utility functions
_global_logger = None

def set_global_logger(logger):
    """Set the global logger for utility functions"""
    global _global_logger
    _global_logger = logger


def add_lock(callback):
    """
    Lock decorator for callback functions, which is
    very helpful for running ROS offline with bag files.
    The lock forces callback functions sequentially,
    so we can show matplotlib plot, etc.

    """

    @wraps(callback)
    def lock_callback(*args, **kwargs):
        if not offline:
            callback(*args, **kwargs)
        else:
            callback_lock_event.wait()
            callback_lock_event.clear()
            callback(*args, **kwargs)
            callback_lock_event.set()

    return lock_callback


class LOGCOLORS:
    DK_GRAY = "\033[30m"
    DK_RED = "\033[31m"
    DK_GREEN = "\033[32m"
    DK_YELLOW = "\033[33m"
    DK_BLUE = "\033[34m"
    DK_PURPLE = "\033[35m"
    DK_CYAN = "\033[36m"
    DK_WHITE = "\033[37m"

    DK_BG_GRAY = "\033[40m"
    DK_BG_RED = "\033[41m"
    DK_BG_GREEN = "\033[42m"
    DK_BG_YELLOW = "\033[43m"
    DK_BG_BLUE = "\033[44m"
    DK_BG_PURPLE = "\033[45m"
    DK_BG_CYAN = "\033[46m"
    DK_BG_WHITE = "\033[47m"

    GRAY = "\033[90m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    PURPLE = "\033[95m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"

    BG_GRAY = "\033[100m"
    BG_RED = "\033[101m"
    BG_GREEN = "\033[102m"
    BG_YELLOW = "\033[103m"
    BG_BLUE = "\033[104m"
    BG_PURPLE = "\033[105m"
    BG_CYAN = "\033[106m"
    BG_WHITE = "\033[107m"

    END = "\033[0m"


def colorlog(color, str):
    return color + str + LOGCOLORS.END


def loginfo(msg):
    if offline:
        tqdm.write(msg)
    else:
        if _global_logger:
            _global_logger.info(msg)
        else:
            print(f"[INFO] {msg}")


def logdebug(msg):
    if offline:
        tqdm.write(colorlog(LOGCOLORS.BLUE, msg))
    else:
        if _global_logger:
            _global_logger.debug(msg)
        else:
            print(f"[DEBUG] {msg}")


def logwarn(msg):
    if offline:
        tqdm.write(colorlog(LOGCOLORS.YELLOW, msg))
    else:
        if _global_logger:
            _global_logger.warning(msg)
        else:
            print(f"[WARN] {msg}")


def logerror(msg):
    if offline:
        tqdm.write(colorlog(LOGCOLORS.RED, msg))
    else:
        if _global_logger:
            _global_logger.error(msg)
        else:
            print(f"[ERROR] {msg}")


def common_parser(description="node"):
    import argparse

    parser = argparse.ArgumentParser(description=description)

    parser.add_argument("--file", type=str, default="", help="ROS bag file")
    parser.add_argument(
        "--start",
        type=float,
        default=None,
        help="start the video from START seconds (default: 0.0)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="duration of the video from START (default: -1)",
    )

    return parser


def read_bag(file, start=None, duration=None, progress=True):
    from rosbags.rosbag2 import Reader
    from rosbags.serde import deserialize_cdr
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    
    # Try ROS2 bag first, fall back to ROS1 if needed
    try:
        with Reader(file) as reader:
            start = start if start is not None else 0
            start_time_ns = int((reader.start_time + start * 1e9))
            end_time_ns = reader.end_time
            
            if duration is None or duration < 0 or duration == float("inf"):
                duration = (end_time_ns - start_time_ns) / 1e9
            else:
                end_time_ns = int(start_time_ns + duration * 1e9)

            if progress:
                pbar = tqdm(total=int(duration), unit="s")
                
            for connection, timestamp, rawdata in reader.messages():
                if timestamp < start_time_ns or timestamp > end_time_ns:
                    continue
                    
                msg_type = get_message(connection.msgtype)
                msg = deserialize_message(rawdata, msg_type)
                
                if progress:
                    elapsed = (timestamp - start_time_ns) / 1e9
                    pbar.update(int(elapsed) - pbar.n)
                    
                yield connection.topic, msg
    except Exception as e:
        # Fall back to ROS1 rosbag
        logwarn(f"Failed to read as ROS2 bag, trying ROS1 format: {e}")
        import rosbag
        
        bag = rosbag.Bag(file)
        start = start if start is not None else 0
        start_time = bag.get_start_time() + start
        end_time = bag.get_end_time()
        if duration is None or duration < 0 or duration == float("inf"):
            duration = end_time - start_time
        else:
            end_time = start_time + duration

        if progress:
            pbar = tqdm(total=int(duration), unit="s")
        for topic, msg, t in bag.read_messages(
            start_time=RclpyTime(seconds=start_time).to_msg(),
            end_time=RclpyTime(seconds=end_time).to_msg(),
        ):
            if progress:
                pbar.update(int(t.to_sec() - start_time) - pbar.n)
            yield topic, msg

        bag.close()


def get_log_dir():
    import os
    import pathlib
    # In ROS2, use ~/.ros/log or ROS_LOG_DIR environment variable
    log_dir = os.environ.get('ROS_LOG_DIR', os.path.expanduser('~/.ros/log'))
    pathlib.Path(log_dir).mkdir(parents=True, exist_ok=True)
    return log_dir


def create_log(suffix, timestamp=None):
    import datetime
    import os

    if timestamp is None:
        # Get current time in seconds
        timestamp = time.time()

    now = datetime.datetime.fromtimestamp(timestamp)
    log_name = now.strftime("%Y-%m-%d-%H-%M-%S-") + suffix

    log_dir = get_log_dir()

    return os.path.join(log_dir, log_name)


def load_nav_data(file, start=0, duration=None, progress=True):
    import gtsam
    from .topics import IMU_TOPIC, DVL_TOPIC, DEPTH_TOPIC

    dvl, depth, imu = [], [], []
    for topic, msg in read_bag(file, start, duration, progress):
        # Convert ROS2 time to seconds
        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if topic == DVL_TOPIC:
            dvl.append(
                (time, msg.velocity.x, msg.velocity.y, msg.velocity.z, msg.altitude)
            )
        elif topic == DEPTH_TOPIC:
            depth.append((time, msg.depth))
        elif topic == IMU_TOPIC:
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            wx = msg.angular_velocity.x
            wy = msg.angular_velocity.y
            wz = msg.angular_velocity.z
            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            qw = msg.orientation.w
            # IMU is -roll90
            y, p, r = (
                gtsam.Rot3.Quaternion(qw, qx, qy, qz)
                .compose(gtsam.Rot3.Roll(np.pi / 2.0))
                .ypr()
            )
            t = msg.linear_acceleration_covariance[0]
            imu.append((time, ax, ay, az, wx, wy, wz, r, p, y, t))

    dvl = np.array(dvl)
    depth = np.array(depth)
    imu = np.array(imu)
    t0 = [a[0, 0] for a in (dvl, depth, imu) if len(a)]
    if not t0:
        return None, None, None
    else:
        t0 = min(t0)

    if len(dvl):
        dvl[:, 0] -= t0
    if len(imu):
        imu[:, 0] -= t0
        imu[:, -1] -= imu[0, -1]
    if len(depth):
        depth[:, 0] -= t0
    return dvl, depth, imu


class CodeTimer(object):
    """Timer class used with `with` statement

    - Disable output by setting CodeTimer.silent = False
    - Change log_func to print/tqdm.write/rospy.loginfo/etc

    with CodeTimer("Some function"):
        some_func()

    """

    silent = False

    def __init__(self, name="Code block"):
        self.name = name

    def __enter__(self):
        """Start measuring at the start of indent"""
        if not CodeTimer.silent:
            self.start = timeit.default_timer()

    def __exit__(self, exc_type, exc_value, traceback):
        """
            Stop measuring at the end of indent. This will run even
            if the indented lines raise an exception.
        """
        if not CodeTimer.silent:
            self.took = timeit.default_timer() - self.start

            if not CodeTimer.silent:
                msg = "{} : {:.5f} s".format(self.name, float(self.took))
                logdebug(msg)
