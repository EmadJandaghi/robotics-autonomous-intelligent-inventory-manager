# Autonomous Intelligent Inventory Management with ROS 2

This project implements a complete **autonomous inventory management system** in ROS 2 using **Gazebo** and **RViz2**. A mobile robot follows a purple line around a shelf, stops at four **Areas Of Interest (AOIs)**, and performs **YOLOv8 object detection** to check the inventory of food and electronic items.

The system ties together:

- Color based perception for line following and AOI detection  
- Navigation to colored AOIs  
- Intelligent inventory checking on upper and lower shelves using deep learning  

---

## Table of Contents

1. [Overview](#overview)  
2. [System Architecture](#system-architecture)  
3. [Features](#features)  
4. [Repository Layout](#repository-layout)  
5. [Dependencies](#dependencies)  
6. [Building the Package](#building-the-package)  
7. [Running the System](#running-the-system)  
8. [Nodes and Scripts](#nodes-and-scripts)  
   - [AOIFollower (`go_to_aoi.py`)](#aoifollower-go_to_aoipy)  
   - [YoloObjectDetection (`object_detection.py`)](#yoloobjectdetection-object_detectionpy)  
   - [InventoryChecking (`inventory_checking.py`)](#inventorychecking-inventory_checkingpy)  
   - [TrajectorySender (`trajectory_sender.py`)](#trajectorysender-trajectory_senderpy)  
9. [Topics and Services](#topics-and-services)
10. [Demonstration](#Demonstration) 

---

## Overview

The goal of this project is to build a **real world inspired autonomous inventory management system** using ROS 2. A simulated robot navigates around a shelf along a **purple floor line**, stops at **four AOIs** (colored blobs: yellow, green, red, blue), and then uses **YOLOv8** to recognize and count the items on the shelf.

At each AOI, the robot:

1. Moves its head to look at the upper shelf and runs object detection.  
2. Moves its head to look at the lower shelf and runs object detection again.  
3. Merges counts from both shelves to compute the total inventory.  
4. Compares the result with the expected stock for that AOI and reports **missing** and **misplaced** items.


---

## System Architecture

High level pipeline:

```text
Camera (RGB)  -->  AOIFollower (line + AOI color detection)
                      |
                      | /aoi_arrived (Bool)
                      v
                InventoryChecking
                      |
                      | start/stop_detection (Trigger)
                      v
                YoloObjectDetection  -->  ObjectCount (topic)
```

---

## Features

* Autonomous navigation along a colored line.

* Visual detection of AOIs using colored blobs.

* Robot head motion control through a joint trajectory action client.

* YOLOv8 based object detection with filtering of noisy classes.

* Custom ObjectCount message to publish object type counts.

*  Inventory checking logic that adapts to the current AOI.

---

## Repository Layout

A typical layout for the final_project package looks like:

final_project/
  final_project/
    __init__.py
    go_to_aoi.py
    object_detection.py
    inventory_checking.py
    trajectory_sender.py
  launch/
    autonomous_inventory.launch.py
  data/
    yolov8n.pt
  package.xml
  setup.py

Key points:

* final_project/ (inner) is the Python package that contains all scripts.

* launch/autonomous_inventory.launch.py starts all core nodes for the inventory mission.

* data/yolov8n.pt holds the YOLOv8 model used for object detection.

* setup.py registers console entry points for ROS 2 executables.


---

## Dependencies

You will need:

* ROS 2 (for example Humble or Foxy, adjust to your environment)

* Python 3

* rclpy, sensor_msgs, std_msgs, geometry_msgs, std_srvs, control_msgs, trajectory_msgs

* cv_bridge, OpenCV (cv2)

* numpy

* ultralytics (for YOLOv8)

* Custom message package custom_msgs that defines:

* Yolov8Inference (and InferenceResult)

* ObjectCount (with fields string[] classes, int32[] counts)

* Make sure your ROS 2 overlay workspace has custom_msgs built and sourced.


---

## building the Package

From your ROS 2 workspace (for example ~/ros2_ws):
cd ~/ros2_ws/src
git clone <this-repo-url> final_project
cd ~/ros2_ws
colcon build
source install/setup.bash

Ensure that the YOLO model file is placed at:
final_project/data/yolov8n.pt

---

## Running the System

1. Launch the simulation

* Start the Gazebo world and robot that exposes the camera topic
/deepmind_robot1/deepmind_robot1_camera/image_raw.

* This depends on your simulation environment (for example, The Construct).

2. Launch the inventory pipeline
ros2 launch final_project autonomous_inventory.launch.py

This starts:
* go_to_aoi (AOIFollower node)
* object_detection (YOLOv8 node)
* inventory_checking (InventoryChecking node)

3. Observe behavior

* The robot will tilt its head downward and begin following the purple line.
* It will automatically sequence through AOIs: yellow, green, red, blue.
* At each AOI, the inventory node moves the head to upper and lower shelves, runs detection, then prints inventory status.

You can visualize topics in RViz2 if desired (camera images, detection results, etc).

---

## Nodes and Scripts
AOIFollower (go_to_aoi.py)

Node name: aoi_follower

This node is responsible for:

* Tilting the head down at startup so the camera sees the floor.

* Following the purple line around the shelf.

* Detecting colored AOI blobs in the upper region of the image.

* Publishing which AOI to target and when the robot has arrived.

Key publishers

* cmd_vel (geometry_msgs/Twist)

* /aoi_arrived (std_msgs/Bool)

* /aoi_choice (std_msgs/String)

Key subscribers

* /deepmind_robot1/deepmind_robot1_camera/image_raw (sensor_msgs/Image)

* /inventory_done (std_msgs/Bool)

Internal behavior

* Uses CvBridge to convert ROS images to OpenCV.

* Converts both crops to HSV and thresholds them with tuned color ranges for:

* Extracts contours and centroids to:

* Keeps a sequence of AOIs ["yellow", "green", "red", "blue"].

Methods of interest

* controller_loop: Main timer loop that decides whether to move, wait for inventory, or reposition.

* tilt_head_down: Uses TrajectorySender to send a head trajectory so the camera faces the floor before line following starts.

* proc_img_go_aoi: Full image processing pipeline for line following and AOI detection.

* go_to_aoi: Precise centering on the AOI using the blob centroid.

* pub_velocities and pub_aoi_velocities: Twists for line following and final AOI approach.

* reposition: Simple base rotation to reacquire the line after finishing inventory.

---

## YoloObjectDetection (object_detection.py)

This node wraps the YOLOv8 model and publishes:

* Raw detections as a Yolov8Inference message.

* Annotated image with bounding boxes.

* Object counts per class.

Key subscribers

* /deepmind_robot1/deepmind_robot1_camera/image_raw (sensor_msgs/Image)

Key publishers

* /Yolov8_Inference (custom_msgs/Yolov8Inference) (List of bounding boxes and class names)

* /inference_result (sensor_msgs/Image) (Annotated image with detection boxes and labels)

* /detected_object_count (custom_msgs/ObjectCount) (Aggregated counts of detected objects)

Services

* start_detection (std_srvs/Trigger)

* stop_detection (std_srvs/Trigger)

Detection pipeline

1. Convert incoming ROS image to BGR with CvBridge.

2. Run self.model(img) from ultralytics.YOLO.

3. Iterate over boxes:

* Extract bounding box coordinates and class id.

* Compute width, height, and center of the box.

* Fill an InferenceResult and append to Yolov8Inference.

4. Filter out:

* Undesired classes: ['dining table', 'keyboard', 'umbrella', 'remote', 'chair'].

* Small cell phone detections with height less than 100 pixels (often false positives).

5. Count remaining classes in a dictionary and publish ObjectCount.

6. Render the detections with results[0].plot() and publish the resulting image.


---

## InventoryChecking (inventory_checking.py)

This node orchestrates the inventory process once the robot reaches an AOI.

Key publishers

* cmd_vel (geometry_msgs/Twist)

* /inventory_done (std_msgs/Bool)

Key subscribers

* /detected_object_count (custom_msgs/ObjectCount)

* /aoi_choice (std_msgs/String)

* /aoi_arrived (std_msgs/Bool)


Service clients

* start_detection (std_srvs/Trigger)

* stop_detection (std_srvs/Trigger)

These clients talk to the object_detection node to start and stop YOLO processing.

Head control

Uses TrajectorySender to command the head joints:

Main logic

When /aoi_arrived receives True and busy is False:

1. Mark busy = True and start perform_inventory in a background thread.

2. Upper shelf:

* Move head to upper shelf pose.

* Set detected_objects = 'upper'.

* Call start_detection, wait for some seconds.

* Call stop_detection.

* In object_callback, received ObjectCount is stored in upper_count.

3. Lower shelf:

* Move head to lower shelf pose.

* Set detected_objects = 'lower'.

* Call start_detection, wait, then stop_detection.

* Callback stores data in lower_count.

4. Merge counts using sum_counts(upper_count, lower_count).

5. Decide expectation based on AOI color:

6. Compute:

* missing: items where counted value is less than expected.

* misplaced: items that are present but not in the allowed set.

7. Log inventory results and publish /inventory_done = True.

8. Release busy = False, so the node is ready for the next AOI.

This design prevents concurrent inventory runs and keeps the main ROS executor responsive by pushing blocking operations into a separate thread.


---

## TrajectorySender (trajectory_sender.py)

Helper class to simplify head motion using a ROS 2 FollowJointTrajectory action client.

Action server

* /deepmind_bot_head_controller/follow_joint_trajectory

Behavior

* On construction, it waits for the action server and raises an error if not available.

* send_trajectory(joint_names, positions, duration_sec):

This is used by both:

* AOIFollower to tilt the head down at the beginning of the mission.

* InventoryChecking to switch between upper and lower shelf viewpoints.


---

## Topics and Services

Topics (publishers)

* cmd_vel (geometry_msgs/Twist)

* /aoi_arrived (std_msgs/Bool)

* /aoi_choice (std_msgs/String)

* /inventory_done (std_msgs/Bool)

* /Yolov8_Inference (custom_msgs/Yolov8Inference)

* /inference_result (sensor_msgs/Image)

* /detected_object_count (custom_msgs/ObjectCount)

Topics (subscribers)

* /deepmind_robot1/deepmind_robot1_camera/image_raw (sensor_msgs/Image)

* /inventory_done (std_msgs/Bool)

* /aoi_choice (std_msgs/String)

* /aoi_arrived (std_msgs/Bool)

* /detected_object_count (custom_msgs/ObjectCount)

Services

* start_detection (std_srvs/Trigger)

* stop_detection (std_srvs/Trigger)


---

## Demonstration

1. Purple Line Following

The robot follows the designated purple path around the shelf, maintaining alignment through continuous visual feedback.

2. AOI Blob Detection and Alignment

The robot identifies colored AOI blobs (yellow, green, red, blue), switches into AOI centering mode, and aligns itself precisely before stopping.

3. YOLO Shelf Scanning

Demonstration of upper and lower shelf scanning during the inventory pipeline using YOLOv8:

Upper shelf detection

Lower shelf detection

Full inventory generation

4. Robot Reacquiring the Line and Searching for Next AOI

After finishing an AOI inventory cycle, the robot rotates, repositions, finds the purple line again, and proceeds to the next AOI.

5. Mission Completion and Shutdown

Once all four AOIs have been visited and checked, the robot performs a safe stop and ends the mission.


A sample image showing YOLOv8 detections overlaid on the camera feed, including bounding boxes and labels.

