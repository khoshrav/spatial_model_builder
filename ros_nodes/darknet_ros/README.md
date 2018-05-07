# Darknet ROS version 2
This version of darknet provides an ROS interface for running the YOLO detection as an ROS node.

## To use:

`git clone --recursive https://github.com/pgigioli/darknet_ros.git`

In yolo_ros.cpp, modify lines:

```
configFilePath = "/home/khoshrav/three_dof_head/darknet_ros/config/yolo.cfg";
weightFilePath = "/home/khoshrav/three_dof_head/darknet_ros/config/yolo.weights";
labelFilePath = "/home/khoshrav/three_dof_head/darknet_ros/config/coco.names";

```

with the correct path to your weights, labels, and cfg.

## Launchfile:

Make sure the usb_cam package is installed and use the yolo_ros.launch with:

`roslaunch darknet_ros yolo_ros.launch`

## Topics:

/found_object - displays "1" or "0" corresponding to whether or not an object has been detected

/YOLO_bboxes  - displays the class label that was detected followed by the confidence score and the bbox coordinates: [Class, prob, x, y, z].

## Dockerfile
Avoid incompatibility issues with this dockerfile that will work out of the box. Docker image includes ubuntu 16.04, ROS kinetic, CUDA 8 and cudnn 6.  Docker image will install darknet_ros and the usb_cam package.

Install docker here: https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/

Install nvidia-docker here: https://github.com/NVIDIA/nvidia-docker

Build docker image:

`docker build -t darknet_ros:latest .`

Run docker container and allow docker access to webcam:

`xhost +`

`nvidia-docker run -it --privileged --network=host -v /tmp/.X11-unix:/tmp/.X11-unix --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -e DISPLAY <image>`

Build darknet_ros node:

`catkin_make`

`source devel/setup.bash`

Launch darknet_ros with usb_cam:

`roslaunch darknet_ros yolo_ros.launch`
