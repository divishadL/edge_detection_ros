# Edge Detection and Point Cloud Generation ROS Nodes

This repository contains ROS nodes for edge detection and point cloud generation using depth and color images from a camera. The provided nodes are designed to work together to perform near to the real-time edge detection and generate a point cloud representation of detected edges.

## Introduction

The provided code consists of two ROS nodes: a server node for edge detection and point cloud generation, and a client node to subscribe to camera images and request edge detection. The server node processes the input image, detects edges, and publishes the edge image as well as the corresponding edge points in the form of a PointCloud message which can be visualised in Rviz. The client node captures images from the camera and sends them to the server for edge detection.

## Prerequisites

- ROS Noetic.
- Python 3.x
- OpenCV, NumPy, and cv_bridge libraries. If already not installed, can be installed with ```sudo apt-get install ros-noetic-cv-bridge

## Installation

### Clone the repository into your ROS workspace and Build the ROS packages:
````
$ mkdir -p ~/catkin_ws/src
$ git clone https://github.com/divishadL/edge_detection_ros.git
$ cd ..
$ catkin_make
$ source/devel/setup.bash


## Running the Nodes

### Server Node (Edge Detection and Point Cloud Generation)

Launch the server node by running:

```rosrun edge_detection edge_detector.py

This will start the server node, which performs edge detection and generates point clouds.

### Client Node (Edge Detection Request)

Launch the client node by running:

```rosrun edge_detection edge_detection_client.py 

This will start the client node, which captures images from the camera and requests edge detection from the server.

## Results

- Detected edges are displayed in green in the edge image.
- The generated point cloud is published on the `/edge_points` topic.
- Due to the processing speed, resulting point cloud in Rviz may be slower than actual traversal of the camera. 

