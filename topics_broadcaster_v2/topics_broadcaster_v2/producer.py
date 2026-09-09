from PIL import Image as PILImage
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image as SensorImage, CameraInfo, NavSatFix
from rclpy.executors import ExternalShutdownException
from . import CONFIGURATION, blue_back

class Producer(Node):

    def __init__(self):
        super().__init__("image_producer")
        
        self.__color_publisher=self.create_publisher(
            msg_type = SensorImage,
            topic = CONFIGURATION["rgb_topic_ugv"],
            qos_profile = 10
        )

        self.__depth_publisher=self.create_publisher(
            msg_type = SensorImage,
            topic = CONFIGURATION["depth_topic_ugv"],
            qos_profile = 10
        )

        self.__intrinsics_publisher=self.create_publisher(
            msg_type = CameraInfo,
            topic = CONFIGURATION["intrinsics_topic_ugv"],
            qos_profile = 10
        )

        self.__gps_publisher=self.create_publisher(
            msg_type = NavSatFix,
            topic = CONFIGURATION["fix_topic_ugv"],
            qos_profile = 10
        )

        self.__heading_publisher = self.create_publisher(
            msg_type = Float32,
            topic = CONFIGURATION["heading_topic_ugv"],
            qos_profile = 10
        )

        self.__timer = self.create_timer(0.1, self.publish) # 10 FPS

        self.__color = "/home/konstantinosf/Projects/gesture_module_v2/gesture_module_LOCAL/frames/high_Come-to-me_338_color.png"
        self.__depth = "/home/konstantinosf/Projects/gesture_module_v2/gesture_module_LOCAL/frames/high_Come-to-me_338_depth.png"

        self.get_logger().info(blue_back("DUMMY PRODUCER (LOCAL DEBUGGING)"))

    def publish(self):

        stamp = self.get_clock().now().to_msg()

        color = np.asarray(PILImage.open(self.__color).convert("RGB"))
        depth = np.asarray(PILImage.open(self.__depth),dtype=np.uint16)
        msg = SensorImage()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_depth_frame"
        msg.height = color.shape[0]
        msg.width = color.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 3 * color.shape[1]
        msg.data = color.tobytes()
        self.__color_publisher.publish(msg)

        msg = SensorImage()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_depth_frame"
        msg.height = depth.shape[0]
        msg.width = depth.shape[1]
        msg.encoding = "16UC1"
        msg.is_bigendian = False
        msg.step = 2 * depth.shape[1]
        msg.data = depth.tobytes()
        self.__depth_publisher.publish(msg)

        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_depth_frame"
        msg.height = color.shape[0]
        msg.width = color.shape[1]
        msg.k = [606.0, 0.0, 423.0, 0.0, 605.0, 231.0, 0.0, 0.0, 1.0] # FR-GESTURE camera intrinsics
        self.__intrinsics_publisher.publish(msg)

        msg = NavSatFix()
        msg.header.stamp = stamp
        (msg.longitude, msg.latitude) = (0.0,0.0)
        self.__gps_publisher.publish(msg)

        msg = Float32()
        msg.data = 0.0 # degrees, magnetic north, clockwise (!)
        self.__heading_publisher.publish(msg)

        self.get_logger().info("Published.")

def main():
    try:
        rclpy.init()
        rclpy.spin(node=Producer())
    except (ExternalShutdownException, KeyboardInterrupt) as e:
        print(e)

if __name__ == '__main__':
    main()