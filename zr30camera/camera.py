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
_PUBLISH_PERIOD_SEC = 0.001

class CameraNode(Node):
    def __init__(self, capture: cv2.VideoCapture, node_name: str =_CAM_NODE_NAME, pub_period: float=_PUBLISH_PERIOD_SEC) -> None:
        super().__init__(node_name)
        self.capture = capture
        self.bridge = CvBridge()
        
        # define video feed publish topic
        self.publisher = self.create_publisher(Image, _CAM_PUB_TOPIC, _QUEUE_SIZE)

        # define publishing frequency and callback function
        self.timer_ = self.create_timer(pub_period, self.capture_image_callback)
        self.count = 0

    def capture_image_callback(self) -> None:
        """
        Captures an image from the camera via RTSP and publishes it as a ROS Image message.
        """
        _, frame = self.capture.read()
        
        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        msg.height = np.shape(frame)[0]
        msg.width = np.shape(frame)[1]
        msg.step = np.shape(frame)[2]*np.shape(frame)[1]
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = _CAM_FRAME_ID

        self.publisher.publish(msg)
        self.get_logger().info(f"Zoom data packet {self.count} published.")
        self.count += 1

def main(args=None):
    capture = cv2.VideoCapture(_CAM_STREAM_URI)

    rclpy.init(args=args)
    camera_publisher = CameraNode(capture=capture)

    rclpy.spin(camera_publisher)

    camera_publisher.destroy_node()
    rclpy.shutdown()
    capture.release()

if __name__ == "__main__":
    main()