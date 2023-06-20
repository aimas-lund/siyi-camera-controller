# Setup
## Install dependencies

Install python dependencies with pip:
```bash
pip3 install -r requirements.txt
```

One of the dependencies is a modified version of the ```siyi_sdk```, which can be accessed in this repository:
```https://github.com/aimas-lund/siyi_sdk_ros2_submodule```. This repo is modified to handle zoom functionalities of the ZR30 camera specifically.

<b>NOTE: The camera firmware must be kept up-to-date as older firmware might not include functionalities such as Absolute Zoom.</b>


## Build ROS2 package

Build with colcon:
```bash
colcon build
```

Source the setup file:
```bash
. install/setup.bash
```

# Run
Run the camera controller and camera stream ROS2 nodes with the launch file:
```bash
ros2 launch zr30camera zr30camera_launch.py
```

If the camera is connected to the network, the camera controller node will automatically connect to the camera and start the stream. If the camera <b>is not</b> connected to the network, the camera controller will timeout after 5 seconds. If the camera is physically connected to the computer, make sure so be a part of the same network as the camera (192.168.144.xxx).

# Control the camera actuators
The ZR30 camera can be controlled via the topics starting with ```/ZR30/set_```. This can either be done with a custom node that publishes, or built-in ```ros2``` functionalities.

For instance, setting the camera zoom level to 20x can be done with the following command:

```bash
ros2 topic pub --once /ZR30/set_zoom_level std_msgs/msg/Float32 'data: 20.0'
```

NOTE: the ```--once``` argument is important, as the camera zoom will otherwise camera controller node will hang.

## Camera control topics

### Zoom control

- ```/ZR30/set_zoom_level```, accepts ```std_msgs/msg/Float32```-type messages.

Control message example:

```bash
ros2 topic pub --once /ZR30/set_zoom_level std_msgs/msg/Float32 'data: 20.0'
```

### Gimbal control

- ```/ZR30/set_gimbal_attitude```, accepts ```geometry_msgs/msg/Vector3Stamped```-type messages, where x = roll, y = pitch and z = yaw.

Control message example:
```bash
ros2 topic pub --once ZR30/set_gimbal_attitude geometry_msgs/msg/Vector3Stamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base'}, vector: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Focus control

- ```/ZR30/set_focus_mode```, accepts ```std_msgs/msg/Int8```-type messages, where -1 = close focus, 0 = hold focus, 1 = far focus, 2 = auto focus.

Control message example:
```bash
ros2 topic pub --once /ZR30/set_zoom_level std_msgs/msg/Int8 'data: 2'
```


# Camera stream
The video stream from the UDP endpoint is parsed to the ```/ZR30/camera_stream``` topic in the ```sensor_msgs/msg/Image``` format.
