## Software
- Ubuntu 18.04
- ROS Melodic

You need to have an installed ROS system with a configured developer environment. The assumption is that your workspace is in your home directory. : ~/catkin_ws.

## The required ROS packages:

### usb_cam
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-drivers/usb_cam
$ cd ..
$ catkin_make
```

After building the package, install the v4l-util Ubuntu package. It is a collection of command-line V4L utilities used by the usb_cam package:
```bash
$ sudo apt install v4l-utils
```

### ROS package for communication with serial port:
```bash
$ sudo apt install ros-melodic-rosserial
```

## Install Face Tracker package
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/MaxTyulin/ros-face-tracker.git
$ cd ..
$ catkin_make
```

## Run
```bash
$ roslaunch face_traker face_traker.launch
```

## Example
*click on the image to watch the video*
[![Face tracking example](http://img.youtube.com/vi/dqZ7WhHHTR0/0.jpg)](http://www.youtube.com/watch?v=dqZ7WhHHTR0 "Face tracking example")

