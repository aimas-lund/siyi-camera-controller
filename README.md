# SIYI ZR30 Camera ROS2 driver

***NOTICE: THIS REPOSITORY IS NOT MAINTAINED***

This repository contains a ROS2 Humble driver for the SIYI ZR30 camera. The driver is based on the SIYI SDK, which can be found here: ```https://github.com/mzahana/siyi_sdk```. The SDK is modified as a python package (see [Install dependencies](#install-dependencies) section).

Following functionalities are implemented:
- Camera stream (/msg/Image topic)
- Camera zoom control (/msg/Float32 topic)
- Camera gimbal control (/msg/Vector3Stamped topic)
- Camera manual- and automatic focus control (/msg/Int8 topic)

# Setup the package
This package is tested with the following setup:
- Ubuntu 22.04
- ROS2 Humble
- Python 3.9 (or higher)
- SIYI ZR30 camera (with up-to-date firmware).
- SIYI SDK installed (see install instructions below)

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

## Configure network
The camera must be connected to the same network as the computer running the ROS2 nodes. Configure the computer network to the following settings:
- IP address: ```192.168.144.xxx```
- Subnet mask: ```255.255.255.0```

The camera will have either of the following addresses:
- ```192.168.144.25``` (default)
- ```192.168.144.26```

The camera will expose the following ports:
- ```8554```: video stream (RTSP protocol)
- ```37260```: camera control (UDP protocol)

The ```siyi_sdk``` will automatically connect to the camera through these ports. However, this information can be useful, if the raw video stream is needed, instead of the /msgs/Image topic.

# Running the package
Run the camera controller and camera stream ROS2 nodes with the launch file:
```bash
ros2 launch zr30camera zr30camera_launch.py
```

If the camera is connected to the network, the camera controller node will automatically connect to the camera and start the stream. If the camera <b>is not</b> connected to the network, the camera controller will timeout after 5 seconds. If the camera is physically connected to the computer, make sure so be a part of the same network as the camera (see [Configure network](#configure-network) section).

# Control the camera actuators
The ZR30 camera can be controlled via the topics starting with ```/ZR30/set_```. This can either be done with a custom node that publishes, or built-in ```ros2``` functionalities.

For instance, setting the camera zoom level to 20x can be done with the following command:

```bash
ros2 topic pub --once /ZR30/set_zoom_level std_msgs/msg/Float32 'data: 20.0'
```

NOTE: the ```--once``` argument is important, as the camera zoom will otherwise camera controller node will hang.

## Camera control topics

### Zoom control

- ```/ZR30/set_zoom_level```, accepts ```std_msgs/msg/Float32```-type messages. The input values are magnification multiplier format, e.g. 20.0x.

Control message example:

```bash
ros2 topic pub --once /ZR30/set_zoom_level std_msgs/msg/Float32 'data: 20.0'
```

### Gimbal control

- ```/ZR30/set_gimbal_attitude```, accepts ```geometry_msgs/msg/Vector3Stamped```-type messages, where x = roll, y = pitch and z = yaw. <b>The input value units are in degrees</b>.

Control message example:
```bash
ros2 topic pub --once ZR30/set_gimbal_attitude geometry_msgs/msg/Vector3Stamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base'}, vector: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Focus control

- ```/ZR30/set_focus```, accepts ```std_msgs/msg/Int8```-type messages, where -1 = close focus, 0 = hold focus, 1 = far focus, 2 = auto focus.

Control message example:
```bash
ros2 topic pub --once /ZR30/set_focus std_msgs/msg/Int8 'data: 2'
```


# Camera stream
The video stream from the UDP endpoint is parsed to the ```/ZR30/camera_stream``` topic in the ```sensor_msgs/msg/Image``` format.
Alternatively, to obtain the raw RTSP feed, remove the camera stream from the launch file, and obtain the video feed from ```192.168.144.25:8554/main.264```.
