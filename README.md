# siyi-camera-controller

## Setup
### Install dependencies

Install python dependencies with pip:
´´´sh
pip3 install -r requirements.txt
´´´

### Build ROS2 package

Build with colcon:
´´´sh
colcon build
´´´

Source the setup file:
´´´sh
. install/setup.bash
´´´

## Run
Run the camera controller ROS2 nodes with the launch file:
´´´sh
ros2 launch zr30camera zr30camera_launch.py
´´´