import abc

class Logger(abc.ABC):
    @abc.abstractmethod
    def info(self, msg):pass
    @abc.abstractmethod
    def warn(self, msg):pass
    @abc.abstractmethod
    def error(self, msg):pass

red_back = lambda x:f"\033[1;101m{x}\033[0;0m"
green_back = lambda x:f"\033[1;102m{x}\033[0;0m"
blue_back = lambda x:f"\033[1;104m{x}\033[0;0m"

red_fore = lambda x:f"\033[1;91m{x}\033[0;0m"
green_fore = lambda x:f"\033[1;92m{x}\033[0;0m"
blue_fore = lambda x:f"\033[1;94m{x}\033[0;0m"

# Available ports: 49152-65535

CONFIGURATION = {

    "fps": 1,

    "server_IP": "127.0.0.1",
    "server_port": 49152,
    "chunk_size": 256,
    # "prelude": b"[===PRELUDE===]",
    "end": b"[===END===]",
    "server_response_size": 16,
    "increment": 16,
    "ok": b"OK",
    "valid": b"VALID",
    "error": b"ERROR",
    "delay": 0.0001,
    "slop": 1e-1,

    "heading_topic_ugv": "/b2/nicla/magnetometer/heading",
    "heading_topic_gs": "/b2/nicla/magnetometer/heading_broadcasted",

    "fix_topic_ugv": "/fix",
    "fix_topic_gs": "/fix_broadcasted",

    "rgb_topic_ugv": "/b2/camera_front_435i/realsense_front_435i/color/image_raw",
    "rgb_topic_gs": "/b2/camera_front_435i/realsense_front_435i/color/image_raw_broadcasted",

    "depth_topic_ugv": "/b2/camera_front_435i/realsense_front_435i/depth/image_rect_raw",
    "depth_topic_gs": "/b2/camera_front_435i/realsense_front_435i/depth/image_rect_raw_broadcasted",

    "intrinsics_topic_ugv": "/b2/camera_front_435i/realsense_front_435i/color/camera_info",
    "intrinsics_topic_gs": "/b2/camera_front_435i/realsense_front_435i/color/camera_info_broadcasted",

}