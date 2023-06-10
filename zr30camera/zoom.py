import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32
from sdk.siyi_sdk import SIYISDK

_GET_ZOOM_TOPIC = "ZR30/get_zoom_level"
_SET_ZOOM_TOPIC = "ZR30/set_zoom_level"
_ZOOM_NODE_NAME = "zoom_node"
_GIMBAL_FRAME_ID = "ZR30_Camera_Zoomer"
_ZR30_SERVER_IP = "192.168.144.25"
_ZR30_SERVER_PORT = 37260
_PUBLISH_PERIOD_SEC = 0.05
_QUEUE_SIZE = 100

class ZoomNode(Node):
    def __init__(self, camera: SIYISDK, node_name: str =_ZOOM_NODE_NAME, pub_period: float=_PUBLISH_PERIOD_SEC) -> None:
        super().__init__(node_name)
        self.camera = camera
        
        # define zoom level publish topic
        self.publisher_ = self.create_publisher(Float32, _GET_ZOOM_TOPIC, _QUEUE_SIZE)

        # define set zoom level command topic
        self.subscriber_ = self.create_subscription(Float32, _SET_ZOOM_TOPIC, self.set_zoom_callback, 10)

        # define publishing frequency and callback function
        self.timer_ = self.create_timer(pub_period, self.get_zoom_callback)
        self.i = 0

    def get_zoom_callback(self) -> None:
        """
        Will use the siyi_sdk to publish the zoom level of the ZR30 Camera.
        """
        msg = Float32()

        msg.data = float(self.camera.getZoomLevel())

        self.publisher_.publish(msg)
        self.get_logger().info(f"Zoom data packet {self.i} published.")
        self.i += 1

    def set_zoom_callback(self, msg: Float32) -> None:
        """
        Will listen for Vector3Stamped messages to toggle the attitude of the gimbal.

        Args:
            msg (Float32): The message containing the zoom level to set.
            msg.data = zoom level
        """
        val = msg.data
        self.awesome_set_zoom_function(val)
        self.get_logger().info(f"Zoom level set to {val}.")

    def awesome_set_zoom_function(zoom_level: float) -> None:
        print("Im awesome!")


def main(args=None):
    camera = SIYISDK(server_ip=_ZR30_SERVER_IP, port=_ZR30_SERVER_PORT)
    camera.connect()

    rclpy.init(args=args)
    node = ZoomNode(camera=camera)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
