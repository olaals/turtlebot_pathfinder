#[![Build Status](https://travis-ci.com/vortexntnu/auv-cv.svg?branch=master)](https://travis-ci.com/vortexntnu/auv-cv)
# AUV-CV
Download this repo to your src-folder in your catkin workspace.


## Installing dependencies
OpenCV >= 3.4.0: https://github.com/opencv/opencv

ROS - Kinetic Kame: http://wiki.ros.org/kinetic

```sh
sudo apt install ros-kinetic-camera-info-manager
sudo apt install ros-kinetic-cv-bridge
```

```sh
sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps
ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator
ros-kinetic-kobuki-ftdi
```
## ROS nodes


### turtlebot
Task 1 of the competition. Uses color filtering together with contour detection by OpenCV to capture the pole(s) of the gate. 
Publishes the midpoint of the detected pole(s) on the topic: `pole_midpoint`.

Usage: 
```bash
$ roslaunch followbot course.launch
```

Topics should be spesified for `[topic]`.

### Path_marker
Uses color filtering together with line detection by OpenCV to capture the direction intended by the guide post. The direction (compared to the heading of the AUV) is buffered using the "sliding window"-method before returning the average.

