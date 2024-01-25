# ROS Project Guide

This document provides instructions on how to set up and use the ROS project for tracking TurtleSim robots.

## Prerequisites

- ROS (Robot Operating System) installed, preferably ROS Noetic.
- `turtlesim` package installed with ROS.
- Python with `requests` library installed for HTTP requests.
- Access to the Flask server running on an EC2 instance or similar setup.

## Setup and Running Instructions

### 0. Run EC2 Server
Connect to EC2 Server and run flask server

### 1. Start ROS Core
Open a terminal and start the ROS master node:
```bash
roscore
```

### 2. Run TurtleSim Node
In a new terminal, launch the TurtleSim simulation:
```bash
rosrun turtlesim turtlesim_node
```

### 3. Spawn Additional Turtles (Optional)
If you need more than one turtle, spawn them using the following command:
```bash
rosservice call /spawn "{x: 5.5, y: 5.5, theta: 0, name: 'turtle2'}"
```
Repeat this step for as many turtles as you need, ensuring each has a unique name.

### 4. Run Custom ROS Node
Navigate to your ROS package directory and run your custom node (turtle number is specified by user):
```bash
rosrun my_robot_package my_robot_node.py _turtle_number:=[turtle_number]
rosrun turtlesim turtle_teleop_key /turtle1/cmd_vel:=/turtle[turtle_number]/cmd_vel
```
If you have multiple turtles, repeat this command in different terminals with the appropriate `turtle_number`.

### 5. Monitor Turtle Movements
Your custom ROS node will track the movements of the TurtleSim robots and send their coordinates to the configured Flask server.

### 6. Verify Data on Flask Server
Check your Flask server's endpoint to verify that it's receiving and correctly processing the data from your ROS node.

### 7. Check plot images for robots created
You can visit http://[EC2 Server Public IP]/5000/[robot_name]

## API Endpoints
### GET `/[robot_name]`
Returns a web page with the plot graph of [robot_name]

### POST `/[robot_name]/[x-coordinates]/[y-coordinates]`
Tracks robot's new coordinates and adds to the plot graph in `/[robot_name]/`.

### GET `/All`
Returns all of the coordinates of the different robots created so far in the server in JSON format.

## Troubleshooting

- Ensure all ROS nodes are running in the correct order.
- If coordinates are not updating, verify the subscription to the `/turtle[number]/pose` topic.
- Check network connectivity if the Flask server is not receiving data.

## Additional Notes

