import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, Int8
from siyi_sdk.siyi_sdk import SIYISDK
from time import sleep

_GET_GIMBAL_ATTITUTE_TOPIC = "ZR30/get_gimbal_attitude"
_GET_ZOOM_TOPIC = "ZR30/get_zoom_level"
_SET_GIMBAL_ATTITUDE_TOPIC = "ZR30/set_gimbal_attitude"
_SET_ZOOM_TOPIC = "ZR30/set_zoom_level"
_SET_FOCUS_TOPIC = "ZR30/set_focus"

_CONTROLLER_NODE_NAME = "zr30_controller_node"
_GIMBAL_FRAME_ID = "ZR30_Camera_Gimbal"

_ZR30_SERVER_IP = "192.168.144.25"
_ZR30_SERVER_PORT = 37260

_PUBLISH_PERIOD_SEC = 0.05
_QUEUE_SIZE = 100
_GIMBAL_KP = 4
_GIMBAL_ERR_THRESH = 5.0

class CameraControllerNode(Node):
    def __init__(self, camera: SIYISDK, node_name: str =_CONTROLLER_NODE_NAME, pub_period: float=_PUBLISH_PERIOD_SEC) -> None:
        super().__init__(node_name)
        self.camera = camera
        
        # define get attribute topics
        self.att_publisher_ = self.create_publisher(Vector3Stamped, _GET_GIMBAL_ATTITUTE_TOPIC, _QUEUE_SIZE)
        self.zoom_publisher_ = self.create_publisher(Float32, _GET_ZOOM_TOPIC, _QUEUE_SIZE)

        # define set attribute topics
        self.att_subscriber_ = self.create_subscription(Vector3Stamped, _SET_GIMBAL_ATTITUDE_TOPIC, self.set_attitude_callback, 10)
        self.zoom_subscriber_ = self.create_subscription(Float32, _SET_ZOOM_TOPIC, self.set_zoom_callback, 10)
        self.focus_subscriber_ = self.create_subscription(Int8, _SET_FOCUS_TOPIC, self.set_focus_callback, 10)

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
        self.camera.setGimbalRotation(yaw, pitch, err_thresh=_GIMBAL_ERR_THRESH, kp=_GIMBAL_KP)


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

        if val == 1.0 or val == 30.0:
            self._request_zoom(val)
        
        self.camera.setZoomLevel(val)
        self.get_logger().info(f"Zoom level set to {val}.")

    def set_focus_callback(self, msg: Int8) -> None:
        """
        Will listen for Int8 messages to toggle the focus of the camera.

        Args:
            msg (Int8): The message containing the focus level to set.
            msg.data = -1 (close shot), 0 (stop focus), 1 (far shot), 2 (auto)
        """
        val = msg.data
        
        if (val == 2):
            self.camera.requestAutoFocus()
            self.get_logger().info(f"Auto focus requested.")
        elif(val == 1):
            self.camera.requestLongFocus()
            self.get_logger().info(f"Long focus requested.")
        elif(val == 0):
            self.camera.requestFocusHold()
            self.get_logger().info(f"Focus hold requested.")
        elif(val == -1):
            self.camera.requestCloseFocus()
            self.get_logger().info(f"Close focus requested.")
        else:
            self.get_logger().error(f"Invalid focus argument {val}.")
            return
        
    def _request_zoom(self, zoom) -> None:
        """
        Will zoom in or out on the camera and return the zoom level.
        Zoom level: min = 1.0, max = 30.0
        Args:
            zoom: whether to zoom in (zoom = 1) or zoom out (zoom = 0)
        """
        cam_zoom = float(self.camera.getZoomLevel())
        print("Initial zoom level", cam_zoom)

        if zoom < cam_zoom:
            while zoom < cam_zoom:
                self.camera.requestZoomOut()
                cam_zoom = float(self.camera.getZoomLevel())
        elif zoom > cam_zoom:
            while zoom > cam_zoom:
                self.camera.requestZoomIn()
                cam_zoom = float(self.camera.getZoomLevel())
        else:
            pass

        # if zoom == 1:
        #     print("Zooming in")        
        #     val = self.camera.requestZoomIn()
        #     sleep(1)
        # elif zoom == -1:
        #     print("Zooming out")
        #     val = self.camera.requestZoomOut()
        #     sleep(1)
        # else:
        #     print("Wrong input to zoom. Input 1 or -1.")
        #     pass

        val = self.camera.requestZoomHold()
        sleep(1)
        cam_zoom = float(self.camera.getZoomLevel())
        sleep(1)

        print("Achieved zoom level: ", cam_zoom)


def main(args=None):
    camera = SIYISDK(server_ip=_ZR30_SERVER_IP, port=_ZR30_SERVER_PORT)
    camera.connect()

    rclpy.init(args=args)
    node = CameraControllerNode(camera=camera)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    camera.disconnect()

if __name__ == "__main__":
    main()
