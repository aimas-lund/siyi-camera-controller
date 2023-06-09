import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32
from siyi_sdk import SIYISDK
from time import sleep

_GET_ZOOM_TOPIC = "ZR30/get_zoom_level"
_SET_ZOOM_TOPIC = "ZR30/set_zoom_level"
_ZOOM_NODE_NAME = "zoom_node"
_GIMBAL_FRAME_ID = "ZR30_Camera_Zoomer"
_PUBLISH_PERIOD_SEC = 0.05
_QUEUE_SIZE = 100

class ZoomNode(Node):
    def __init__(self, camera: SIYISDK, node_name: str =_ZOOM_NODE_NAME, pub_period: float=_PUBLISH_PERIOD_SEC) -> None:
        super.__init__(node_name)
        self.camera = camera
        
        # define zoom level publish topic
        self.publisher = self.create_publisher(Float32, _GET_ZOOM_TOPIC, _QUEUE_SIZE)

        # define set zoom level command topic
        self.subscriber = self.create_subscription(Float32, _SET_ZOOM_TOPIC, self.subscribe_callback, 10)

        # define publishing frequency and callback function
        self.timer_ = self.create_timer(pub_period, self.get_zoom_callback)
        self.count = 0

    def get_zoom_callback(self) -> None:
        """
        Will use the siyi_sdk to publish the zoom level of the ZR30 Camera.
        """
        msg = Float32()

        msg.data = self.camera.getZoomLevel()
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = _GIMBAL_FRAME_ID

        self.publisher.publish(msg)
        self.get_logger().info(f"Zoom data packet {self.count} published.")
        self.count += 1

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

    def awesome_set_zoom_function(self, zoom):
        """
        Will zoom in or out on the camera and return the zoom level.
        Zoom level: min = 1.0, max = 30.0

        Args:
            zoom: whether to zoom in (zoom = 1) or zoom out (zoom = 0)
        """
        cam_zoom = float(self.camera.getZoomLevel())
        print("Initial zoom level", cam_zoom)

        if zoom == 1:
            print("Zooming in")        
            val = self.camera.requestZoomIn()
            sleep(1)
        elif zoom == -1:
            print("Zooming out")
            val = self.camera.requestZoomOut()
            sleep(1)
        else:
            print("Wrong input to zoom. Input 1 or -1.")
            pass

        val = self.camera.requestZoomHold()
        sleep(1)
        cam_zoom = float(self.camera.getZoomLevel())
        sleep(1)

        print("Achieved zoom level: ", cam_zoom)
        return cam_zoom #let's figure out where/how we want this info


def main(args=None):
    camera = SIYISDK(server_ip="192.168.144.25", port=37260)
    camera.connect()

    rclpy.init(args=args)
    node = ZoomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
