import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix, Image, CameraInfo
from rclpy.executors import ExternalShutdownException
import socket
from . import Logger, blue_fore, blue_back, CONFIGURATION, red_fore, green_fore
import pickle
import time
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.qos import qos_profile_sensor_data
import math

def send_TCP(chunk:bytes,                   #
            logger:Logger|None=None,        #
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
        chunk_size = client.send(chunk)
        if chunk_size != len(chunk):
            if logger is not None:
                logger.error(f"{prefix} {red_fore('Data truncated by client (client error)')}")
            return config["error"]
            
        response = client.recv(config["server_response_size"])
        return response

    except ConnectionRefusedError:
        if logger is not None:
            logger.error(f"{prefix} {red_fore('Connection refused (server error)')}")
        return config["error"]

    except ConnectionResetError:
        if logger is not None:
            logger.error(f"{prefix} {red_fore('Connection reset (server error)')}")
        return config["error"]


class Receiver(Node, Logger):

    def __init__(self, config:dict):
        super().__init__("receiver_client")
        self.__config = config

        self.__time_synchronizer = ApproximateTimeSynchronizer(
            fs=[
                Subscriber(node=self, msg_type=Image, topic=self.__config["rgb_topic_ugv"]), 
                Subscriber(node=self, msg_type=Image, topic=self.__config["depth_topic_ugv"]), 
                Subscriber(node=self, msg_type=CameraInfo, topic=self.__config["intrinsics_topic_ugv"]),
                Subscriber(node=self, msg_type=NavSatFix, topic=self.__config["fix_topic_ugv"], qos_profile=qos_profile_sensor_data),
                Subscriber(node=self, msg_type=Float32, topic=self.__config["heading_topic_ugv"])
            ],
            queue_size=10,
            slop=self.__config["slop"],
            allow_headerless=True
        )

        self.__time_synchronizer.registerCallback(self.__callback)

        self.info(blue_back("RUNNING RECEIVER (CLIENT) ON JETSON"))

        self.__previous = None
        self.__msg_id = 0

    def __fps_filter(self):
        fps = self.__config["fps"]
        if self.__previous is not None:
            appr_fps = 1 / (time.time() - self.__previous)
            if appr_fps > fps:
                self.warn(f"Callback rejected: approximated FPS = {appr_fps} > {fps}")
                return False
        return True

    def __update(self):
        self.__previous = time.time()

    def info(self, msg):
        self.get_logger().info(msg)
    
    def warn(self, msg):
        self.get_logger().warning(msg)

    def error(self, msg):
        self.get_logger().error(msg)

    def __callback(self, color:Image, depth:Image, intrinsics:CameraInfo, fix:NavSatFix, heading:Float32):
        if not self.__fps_filter():
            return
        self.__msg_id += 1
        msg_bytes = pickle.dumps({
            "id": self.__msg_id,
            "color": color,
            "depth": depth,
            "intrinsics": intrinsics,
            "fix": fix,
            "heading": heading
        }) + self.__config["end"]
        chunk_size = self.__config["chunk_size"]
        total_chunks = math.ceil(len(msg_bytes) / chunk_size)
        prefix = f"\033[0;0m[ID: {blue_fore(self.__msg_id)}]"
        self.info(f"{prefix} [Total size: {len(msg_bytes)}B total size] [End indicator size: {len(self.__config['end'])}B] [Chunks: {blue_fore(total_chunks)} x {chunk_size}B] [{blue_fore(self.__config['server_IP'])}:{blue_fore(self.__config['server_port'])}]")
        self.__update()
        for i in range(total_chunks):
            chunk = msg_bytes[i*chunk_size:(i+1)*chunk_size]
            while True:
                response = send_TCP(chunk=chunk,logger=self,prefix=prefix,config=self.__config)        
                if response == self.__config["error"]:
                    self.warn(f"{prefix} Error during transmitting chunk {i+1}/{total_chunks}, retry ...")
                    time.sleep(self.__config["delay"])
                else:
                    break # ok, invalid, valid
        if response == self.__config["valid"]:
            self.info(f"{prefix} {green_fore('Success')}")
        elif response == self.__config["invalid"]:
            self.error(f"{prefix} {red_fore('Failure')}")
        else:
            self.warn(f"{prefix} {red_fore('Uknown reponse code')} {response}")


def main():
    try:
        rclpy.init()
        rclpy.spin(node=Receiver(config=CONFIGURATION))
    except (ExternalShutdownException, KeyboardInterrupt) as e:
        print(e)

if __name__ == '__main__':
    main()