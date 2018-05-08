## Building Spatial Models
This project builds a model of the position of different objects in the scene using a 3 degree of freedom robotic head with a RGB-D camera mounted on it.

## Applications
These models can be used as search priors to find objects, as goal distributions to show the common state of the world, or to learn inter-object relationships

## Implementation
The code is written in C++ using ROS

## Repository Structure
- keras-yolo
  This is a python implementation of YOLO to train the weights of the model
- ros_nodes
  Contains source code to run the model building and search
- Report: Building Spatial Models.pdf
  This paper summarizes the work done in the project and the implementation details
- head_search_building.mp4
  Recording of a sample model being built
