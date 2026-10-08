import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix, Image, CameraInfo
from rclpy.executors import ExternalShutdownException
import socket
from . import Logger, blue_fore, blue_back, CONFIGURATION, red_fore, green_fore, red_back, compress_image
import pickle
import time
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.qos import qos_profile_sensor_data
import math
import numpy as np
import cv2

def send_TCP(msg_bytes:bytes,               #
            logger:Logger,                  #
            prefix:str="",                  #
            config:dict=CONFIGURATION,      #
            ) -> bytes:

    try:

        address = config["server_IP"]
        port = config["server_port"]

        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, config["chunk_size"]+config["increment"])
        client.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, config["server_response_size"]+config["increment"])
        client.connect((address, port))

        chunk_size = config["chunk_size"]
        total_chunks = math.ceil(len(msg_bytes) / chunk_size)

        logger.info(f"{prefix} [Total size: {len(msg_bytes)}B total size] [End indicator size: {len(config['end'])}B] [Chunks: {blue_fore(total_chunks)} x {chunk_size}B] [{blue_fore(config['server_IP'])}:{blue_fore(config['server_port'])}]")

        total_send = 0

        for i in range(total_chunks):

            chunk = msg_bytes[i*chunk_size:(i+1)*chunk_size]
            send_size = client.send(chunk)
            total_send += send_size
            response = client.recv(config["server_response_size"])
            print(f"{prefix} Sent: [{i+1}/{total_chunks}]", end="\r")

        print()

        while True: # avoid closing connection ("[Errno 104] Connection reset by peer")
            if response == config["valid"]:
                logger.info(f"{prefix} {green_fore('Success')}")
                break
            elif response == config["invalid"]:
                logger.error(f"{prefix} {red_fore('Failure')}")
                break
            elif response == b"":
                logger.warn(f"{prefix} Final response (valid or invalid) not received")
                break
            response = client.recv(config["server_response_size"])

        if total_send != len(msg_bytes):
            logger.error(f"{prefix} {red_fore('Data truncated by client')}")

        return response

    except ConnectionRefusedError:
        logger.error(f"{prefix} {red_fore('Connection refused (server error)')}")
        return config["error"]

    except ConnectionResetError:
        logger.error(f"{prefix} {red_fore('Connection reset (server error)')}")
        return config["error"]

    except BrokenPipeError:
        logger.error(f"{prefix} {red_fore('Connection closed by server (server error)')}")
        return config["error"]

    except BaseException as e:
        logger.error(f"{prefix} {red_back(str(e))}")
        return config["error"]

    finally:
        logger.info(f"{prefix} Connection closed")
        client.close()


class Receiver(Node, Logger):

    def __init__(self, config:dict):
        super().__init__("receiver_client")
        self.__config = config

        self.__time_synchronizer = ApproximateTimeSynchronizer(
            fs=[
                Subscriber(node=self, msg_type=Image, topic=self.__config["rgb_topic_ugv"]),
                Subscriber(node=self, msg_type=Image, topic=self.__config["depth_topic_ugv"]),
                Subscriber(node=self, msg_type=Image, topic=self.__config["flir_topic_ugv"]),
                # Subscriber(node=self, msg_type=CameraInfo, topic=self.__config["intrinsics_topic_ugv"]),
                # Subscriber(node=self, msg_type=NavSatFix, topic=self.__config["fix_topic_ugv"], qos_profile=qos_profile_sensor_data),
                Subscriber(node=self, msg_type=Float32, topic=self.__config["heading_topic_ugv"])
            ],
            queue_size=1,
            slop=self.__config["slop"],
            allow_headerless=True
        )

        self.__time_synchronizer.registerCallback(self.__callback)

        self.info(blue_back("RUNNING RECEIVER (CLIENT) ON JETSON"))

        self.__previous = None
        self.__msg_id = 0

        self.declare_parameter("rgb_compression_quality", self.__config["rgb_quality"])
        self.declare_parameter("depth_compression_quality", self.__config["depth_quality"])
        self.declare_parameter("flir_compression_quality", self.__config["flir_quality"])

    def __fps_filter(self):
        fps = self.__config["fps"]
        if self.__previous is not None:
            appr_fps = 1 / (time.time() - self.__previous)
            if appr_fps > fps:
                self.warn(f"Callback rejected: approximated FPS = {appr_fps} > {fps}")
                return False
        return True

    def info(self, msg):
        self.get_logger().info(msg)
    
    def warn(self, msg):
        self.get_logger().warning(msg)

    def error(self, msg):
        self.get_logger().error(msg)

    # Depends on the image encoding
    def __array_from_image(self, img:Image):

        # RealSense
        yuyv = np.frombuffer(img.data, dtype=np.uint8)
        yuyv = yuyv.reshape((img.height, img.width, 2))
        # # bgr = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)
        # # color_array = bgr.reshape((color_image.height, color_image.width, 3)) # BGR
        rgb = cv2.cvtColor(yuyv, cv2.COLOR_YUV2RGB_YUY2)
        color_array = rgb.reshape((img.height, img.width, 3)) # RGB

        # Dummy test
        # color_array = np.asarray(img.data, dtype=np.uint8).reshape((img.height, img.width, 3)) # H x W x 3
        # color_array = cv2.cvtColor(color_array, cv2.COLOR_BGR2RGB) # RGB

        return color_array

    def __decompose_image_message(self, img:Image) -> dict:
        self.warn("YUV2 was converted to RGB8, message attributes may not be accurate")
        return {
            "header": img.header,
            "height": img.height,
            "width": img.width,
            # "encoding": img.encoding,
            "encoding": "rgb8",
            "is_bigendian": img.is_bigendian,
            # "step": img.step,
            "step": 3 * img.width,
            "data": compress_image(self.__array_from_image(img), self.get_parameter("rgb_compression_quality").get_parameter_value().integer_value)
        }

    def __decompose_depth_message(self, depth:Image) -> dict:
        # Decode depth
        depth_array = np.frombuffer(depth.data, dtype=np.uint16).reshape((depth.height, depth.width)) # mm
        # Normalize depth
        max_depth = depth_array.max()
        depth_normalized = depth_array / max_depth * 255
        depth_discretized = depth_normalized.round() # error 300 mm (!?), grayscaled
        # Compress
        depth_comp = compress_image(depth_discretized, self.get_parameter("depth_compression_quality").get_parameter_value().integer_value)
        return {
            "header": depth.header,
            "height": depth.height,
            "width": depth.width,
            "encoding": depth.encoding, # 16UC1
            "is_bigendian": depth.is_bigendian,
            "step": depth.step,
            "data": depth_comp,
            "max_depth": max_depth,
        }

    def __decompose_flir_message(self, flir:Image) -> dict:
        array = np.frombuffer(flir.data, dtype=np.uint8).reshape((flir.height, flir.width, 3))
        return {
            "header": flir.header,
            "height": flir.height,
            "width": flir.width,
            "encoding": flir.encoding, # RGB-8
            "is_bigendian": flir.is_bigendian,
            "step": flir.step,
            "data": compress_image(array, self.get_parameter("flir_compression_quality").get_parameter_value().integer_value)
        }

    def __callback(self, color:Image, depth:Image, flir:Image, #intrinsics:CameraInfo, fix:NavSatFix, 
                   heading:Float32):
        if not self.__fps_filter():
            return
        self.__msg_id += 1
        msg_bytes = pickle.dumps({
            "id": self.__msg_id,
            "color": self.__decompose_image_message(color),
            "depth": self.__decompose_depth_message(depth),
            "flir": self.__decompose_flir_message(flir),
            "heading": heading,
            # "intrinsics": intrinsics,
            # "fix": fix,
        }) + self.__config["end"]
        self.__previous = time.time()
        prefix = f"\033[0;0m[ID: {blue_fore(self.__msg_id)}]"
        response = send_TCP(msg_bytes=msg_bytes, logger=self, prefix=prefix, config=self.__config)
        if response != self.__config["valid"]:
            self.__previous = None

def main():
    try:
        rclpy.init()
        rclpy.spin(node=Receiver(config=CONFIGURATION))
    except (ExternalShutdownException, KeyboardInterrupt) as e:
        print(e)

if __name__ == '__main__':
    main()
