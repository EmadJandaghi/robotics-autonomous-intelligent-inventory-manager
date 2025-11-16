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
10. [Inventory Logic and Scenarios](#inventory-logic-and-scenarios)  
11. [Future Improvements](#future-improvements)

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
