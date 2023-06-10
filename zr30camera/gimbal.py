import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from siyi_sdk.siyi_sdk import SIYISDK

_GIMBAL_GET_ATTITUTE_TOPIC = "get_gimbal_attitude"
_GIMBAL_SET_ATTITUDE_TOPIC = "set_gimbal_attitude"
_GIMBAL_NODE_NAME = "gimbal_node"
_GIMBAL_FRAME_ID = "ZR30_Camera_Gimbal"
_ZR30_SERVER_IP = "192.168.144.25"
_ZR30_SERVER_PORT = 37260
_PUBLISH_PERIOD_SEC = 0.05
_QUEUE_SIZE = 100

class GimbalNode(Node):
    def __init__(self, camera: SIYISDK, node_name: str =_GIMBAL_NODE_NAME, pub_period: float=_PUBLISH_PERIOD_SEC) -> None:
        super.__init__(node_name)
        self.camera = camera
        
        # define attitude publish topic
        self.publisher_ = self.create_publisher(Vector3Stamped, _GIMBAL_GET_ATTITUTE_TOPIC, _QUEUE_SIZE)

        # define set attitude command topic
        self.subscriber_ = self.create_subscription(Vector3Stamped, _GIMBAL_SET_ATTITUDE_TOPIC, self.subscribe_callback, 10)

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

        self.publisher_.publish(msg)
        self.get_logger().info(f"Gimbal data packet {self.i} published.")
        self.i += 1

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
        self.camera.setGimbalRotation(yaw, pitch)
        self.get_logger().info(f"Gimbal attitude set to ({pitch}, {yaw}) (pitch, yaw).")


def main(args=None):
    camera = SIYISDK(server_ip=_ZR30_SERVER_IP, port=_ZR30_SERVER_PORT)
    camera.connect()

    rclpy.init(args=args)
    node = GimbalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
