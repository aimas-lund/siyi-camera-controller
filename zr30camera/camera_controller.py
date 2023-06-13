import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
from siyi_sdk.siyi_sdk import SIYISDK
from time import sleep

_GIMBAL_GET_ATTITUTE_TOPIC = "ZR30/get_gimbal_attitude"
_GIMBAL_SET_ATTITUDE_TOPIC = "ZR30/set_gimbal_attitude"
_GET_ZOOM_TOPIC = "ZR30/get_zoom_level"
_SET_ZOOM_TOPIC = "ZR30/set_zoom_level"
_CONTROLLER_NODE_NAME = "zr30_controller_node"
_GIMBAL_FRAME_ID = "ZR30_Camera_Gimbal"

_ZR30_SERVER_IP = "192.168.144.25"
_ZR30_SERVER_PORT = 37260
_PUBLISH_PERIOD_SEC = 0.05
_QUEUE_SIZE = 100

class CameraControllerNode(Node):
    def __init__(self, camera: SIYISDK, node_name: str =_CONTROLLER_NODE_NAME, pub_period: float=_PUBLISH_PERIOD_SEC) -> None:
        super().__init__(node_name)
        self.camera = camera
        
        # define get attribute topics
        self.att_publisher_ = self.create_publisher(Vector3Stamped, _GIMBAL_GET_ATTITUTE_TOPIC, _QUEUE_SIZE)
        self.zoom_publisher_ = self.create_publisher(Float32, _GET_ZOOM_TOPIC, _QUEUE_SIZE)

        # define set attribute topics
        self.att_subscriber_ = self.create_subscription(Vector3Stamped, _GIMBAL_SET_ATTITUDE_TOPIC, self.set_attitude_callback, 10)
        self.zoom_subscriber_ = self.create_subscription(Float32, _SET_ZOOM_TOPIC, self.set_zoom_callback, 10)

        # define publishing frequency and callback function
        self.timer_ = self.create_timer(pub_period, self.get_attitude_callback)
        self.i = 0

    def get_attitude_callback(self) -> None:
        """
        Will use the siyi_sdk to publish the current roll, pitch and yaw of the ZR30 Camera.
        """
        msg = Vector3Stamped()

        roll, pitch, yaw = self.camera.getAttitude()
        msg.vector.x = roll
        msg.vector.y = pitch
        msg.vector.z = yaw
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = _GIMBAL_FRAME_ID

        self.att_publisher_.publish(msg)


    def set_attitude_callback(self, msg: Vector3Stamped) -> None:
        """
        Will listen for Vector3Stamped messages to toggle the attitude of the gimbal.

        Args:
            msg (Vector3Stamped): The message containing the attitude to set in the following format
            msg.vector.x = roll (will be ignored)
            msg.vector.y = pitch
            msg.vector.z = yaw
        """
        
        pitch = msg.vector.y
        yaw = msg.vector.z
        self.get_logger().info(f"Gimbal attitude set to ({pitch}, {yaw}) (pitch, yaw).")
        self.camera.setGimbalRotation(yaw, pitch, err_thresh=5.0, kp=4)


    def get_zoom_callback(self) -> None:
        """
        Will use the siyi_sdk to publish the zoom level of the ZR30 Camera.
        """
        msg = Float32()
        zoom_level = float(self.camera.getZoomLevel())
        msg.data = zoom_level

        self.zoom_publisher_.publish(msg)
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


    def awesome_set_zoom_function(self, zoom_level: float) -> None:
        val = self.camera.requestZoomIn()
        sleep(1)
        val = self.camera.requestZoomHold()
        sleep(1)

        val = self.camera.requestZoomOut()
        sleep(1)
        val = self.camera.requestZoomHold()
        sleep(1)


def main(args=None):
    camera = SIYISDK(server_ip=_ZR30_SERVER_IP, port=_ZR30_SERVER_PORT)
    camera.connect()

    rclpy.init(args=args)
    node = CameraControllerNode(camera=camera)
    rclpy.spin(node)

    # Shutdown procedures after node is killed
    node.destroy_node()
    rclpy.shutdown()
    camera.disconnect()

if __name__ == "__main__":
    main()
