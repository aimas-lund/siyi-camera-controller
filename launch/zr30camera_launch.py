import launch
import launch_ros.actions

_PACKAGE_NAME = 'zr30camera'
_GIMBAL_NODE_NAME = 'zr30_gimbal'
_ZOOM_NODE_NAME = 'zr30_zoom'
_CAMERA_NODE_NAME = 'zr30_camera'
_GIMBAL_EXECUTABLE = 'gimbal'
_ZOOM_EXECUTABLE = 'zoom'
_CAMERA_EXECUTABLE = 'camera'

def generate_launch_description() -> launch.LaunchDescription:
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package=_PACKAGE_NAME,
            executable=_GIMBAL_EXECUTABLE,
            name=_GIMBAL_NODE_NAME,),

        launch_ros.actions.Node(
            package=_PACKAGE_NAME,
            executable=_ZOOM_EXECUTABLE,
            name=_ZOOM_NODE_NAME,),

        launch_ros.actions.Node(
            package=_PACKAGE_NAME,
            executable=_CAMERA_EXECUTABLE,
            name=_CAMERA_NODE_NAME,)
  ])