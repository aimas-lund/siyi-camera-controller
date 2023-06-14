import cv2
import rclpy
import numpy as np 

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

_CAM_STREAM_URI = "rtsp://192.168.144.25:8554/main.264"
_CAM_NODE_NAME = "camera_node"
_CAM_PUB_TOPIC = "ZR30/camera_stream"
_CAM_FRAME_ID = "ZR30_Camera_Capture"
_QUEUE_SIZE = 100
_PUBLISH_PERIOD_SEC = 0.01

class CameraStreamNode(Node):
    def __init__(self, capture: cv2.VideoCapture, node_name: str =_CAM_NODE_NAME, pub_period: float=_PUBLISH_PERIOD_SEC) -> None:
        super().__init__(node_name)
        self.capture = capture
        self.bridge = CvBridge()
        
        # define video feed publish topic
        self.publisher = self.create_publisher(Image, _CAM_PUB_TOPIC, _QUEUE_SIZE)

        # define publishing frequency and callback function
        self.timer_ = self.create_timer(pub_period, self.capture_image_callback)
        self.i = 0

    def capture_image_callback(self) -> None:
        """
        Captures an image from the camera via RTSP and publishes it as a ROS Image message.
        """
        _, frame = self.capture.read()
        shape = np.shape(frame)

        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        msg.header.frame_id = _CAM_FRAME_ID
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.height = shape[0]
        msg.width = shape[1]
        msg.step = shape[2]*shape[1]

        self.publisher.publish(msg)
        self.i += 1


def main(args=None):
    capture = cv2.VideoCapture(_CAM_STREAM_URI)

    rclpy.init(args=args)
    camera_publisher = CameraStreamNode(capture=capture)

    rclpy.spin(camera_publisher)

    camera_publisher.destroy_node()
    rclpy.shutdown()
    capture.release()

if __name__ == "__main__":
    main()