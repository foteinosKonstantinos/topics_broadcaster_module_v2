import abc
import cv2

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

def compress_image(img, quality):
    res = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not res[0]:
        raise ValueError("Compression failed")
    return res[1]

def decompress_color(data):
    return cv2.imdecode(data, cv2.IMREAD_COLOR)

def decompress_gray(data):
    return cv2.imdecode(data, cv2.IMREAD_GRAYSCALE)


# Available ports: 49152-65535

CONFIGURATION = {

    "fps": 2,

    "server_IP": "127.0.0.1", # TODO
    "server_port": 49152,
    "chunk_size": 4096,
    # "prelude": b"[===PRELUDE===]",
    "end": b"[===END===]",
    "server_response_size": 16,
    "increment": 16,
    "ok": b"OK",
    "valid": b"VALID",
    "error": b"ERROR",
    "invalid": b"INVALID",
    "delay": 0,
    "slop": 1,

    "heading_topic_ugv": "/b2/nicla/magnetometer/heading",
    "heading_topic_gs": "/b2/nicla/magnetometer/heading_broadcasted",

    # "fix_topic_ugv": "/fix",
    # "fix_topic_gs": "/fix_broadcasted",

    "rgb_topic_ugv": "/b2/camera_front_435i/realsense_front_435i/color/image_raw",
    "rgb_topic_gs": "/b2/camera_front_435i/realsense_front_435i/color/image_raw_broadcasted",
    "rgb_quality": 80, # percentage

    "depth_topic_ugv": "/b2/camera_front_435i/realsense_front_435i/aligned_depth_to_color/image_raw",
    "depth_topic_gs": "/b2/camera_front_435i/realsense_front_435i/aligned_depth_to_color/image_raw_broadcasted",
    "depth_quality": 100, # percentage

    "flir_topic_ugv": "/b2/camera_flir/image_raw",
    "flir_topic_gs": "/b2/camera_flir/image_raw_broadcasted",
    "flir_quality": 80, # percentage

    # "intrinsics_topic_ugv": "/b2/camera_front_435i/realsense_front_435i/color/camera_info",
    # "intrinsics_topic_gs": "/b2/camera_front_435i/realsense_front_435i/color/camera_info_broadcasted",

}