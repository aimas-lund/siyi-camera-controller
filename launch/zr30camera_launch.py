import launch
import launch_ros.actions

_PACKAGE_NAME = 'zr30camera'
_CAMERA_CONTROLLER_NAME = 'zr30_controller'
_CAMERA_STREAM_NODE_NAME = 'zr30_stream'
_CAMERA_CONTROLLER_EXECUTABLE = 'controller'
_CAMERA_STREAM_EXECUTABLE = 'stream'

def generate_launch_description() -> launch.LaunchDescription:
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package=_PACKAGE_NAME,
            executable=_CAMERA_CONTROLLER_EXECUTABLE,
            name=_CAMERA_CONTROLLER_NAME,),

        launch_ros.actions.Node(
            package=_PACKAGE_NAME,
            executable=_CAMERA_STREAM_EXECUTABLE,
            name=_CAMERA_STREAM_NODE_NAME,)
  ])