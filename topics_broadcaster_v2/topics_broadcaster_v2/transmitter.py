import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix, Image, CameraInfo
from rclpy.executors import ExternalShutdownException
import socket
from . import Logger, CONFIGURATION, green_fore, blue_back, blue_fore, red_back
import pickle
from typing import Callable

class Server_Chunked_TCP:

    def __init__(self,
                callback:Callable[[dict], int],     # Function to apply to the message(s)
                logger:Logger|None=None,
                max_connections:int=1,              # Maxinum waiting connections
                config:dict=CONFIGURATION,
                prefix:str="\033[0;0m[TCP SERVER]"
                ):

        self.__callback = callback
        self.__logger = logger
        self.__max_connections = max_connections
        self.__prefix = prefix
        self.__config = config

        self.__message = None

    def start(self) -> None:

        address, port = self.__config["server_IP"], self.__config["server_port"]

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # https://pubs.opengroup.org/onlinepubs/009695399/functions/setsockopt.html
        server.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.__config["server_response_size"]+self.__config["increment"])
        server.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.__config["chunk_size"]+self.__config["increment"])
        server.bind((address, port))
        server.listen(self.__max_connections)
        if self.__logger is not None:
            self.__logger.info(f"{self.__prefix} {green_fore('\u25CF Activated')} @ {blue_fore(address)}:{blue_fore(port)} [in. buffer: {self.__config['chunk_size']}] [max conn.: {self.__max_connections}]")

        while True:

            connection, client_address = server.accept()
            chunk = connection.recv(self.__config["chunk_size"])

            if self.__message is None:
                self.__message = chunk
            else:
                self.__message = self.__message + chunk

            if chunk.endswith(self.__config["end"]):
                try:
                    self.__message = self.__message[len(self.__config["end"]):] # Remove indicator
                    data = pickle.loads(self.__message)
                    msg_id = self.__callback(data)
                    if self.__logger is not None:
                        self.__logger.info(f"{self.__prefix} {green_fore('Success')}: Msg. {blue_fore(msg_id)} parsed (last conn.: {blue_fore(client_address[0])}:{blue_fore(client_address[1])}).")
                    _ = connection.send(self.__config["valid"])
                except pickle.UnpicklingError as e:
                    if self.__logger is not None:
                        self.__logger.error(f"{self.__prefix} Error: '{red_back(e)}' (last conn.: {blue_fore(client_address[0])}:{blue_fore(client_address[1])}).")
                    _ = connection.send(self.__config["error"])
                self.__message = None
            else:
                _ = connection.send(self.__config["ok"])

            connection.close()


class Transmitter(Node, Logger):

    def __init__(self, config:dict):
        super().__init__("transmitter_server")
        self.__config = config
        
        self.__heading_publisher=self.create_publisher(
            msg_type = Float32,
            topic = self.__config["heading_topic_gs"],
            qos_profile = 10,
        )

        self.__fix_publisher=self.create_publisher(
            msg_type = NavSatFix,
            topic = self.__config["fix_topic_gs"],
            qos_profile = 10,
        )

        self.__intrinsics_publisher=self.create_publisher(
            msg_type = CameraInfo,
            topic = self.__config["intrinsics_topic_gs"],
            qos_profile = 10,
        )

        self.__rgb_publisher=self.create_publisher(
            msg_type = Image,
            topic = self.__config["rgb_topic_gs"],
            qos_profile = 10,
        )

        self.__depth_publisher=self.create_publisher(
            msg_type = Image,
            topic = self.__config["depth_topic_gs"],
            qos_profile = 10,
        )

        self.info(blue_back("RUNNING TRANSMITTER (SERVER) ON GROUND STATION"))

        self.__server = Server_Chunked_TCP(callback=self.__callback,logger=self,config=self.__config)
        self.__server.start()

    def info(self, msg):
        self.get_logger().info(msg)

    def warn(self, msg):
        self.get_logger().warning(msg)

    def error(self, msg):
        self.get_logger().error(msg)

    def __callback(self, data:dict) -> int:
        self.__heading_publisher.publish(data["heading"])
        self.__rgb_publisher.publish(data["color"])
        self.__depth_publisher.publish(data["depth"])
        self.__fix_publisher.publish(data["fix"])
        self.__intrinsics_publisher.publish(data["intrinsics"])
        return data["id"]

def main():
    try:
        rclpy.init()
        rclpy.spin(node=Transmitter(config=CONFIGURATION))
    except (ExternalShutdownException, KeyboardInterrupt) as e:
        print(e)

if __name__ == '__main__':
    main()