# COMP3431

This repository contains ROS (Robot Operating System) code for controlling a robot's behavior using sensor data, image processing, and navigation techniques.

## ColorPublisher Node

The `ColorPublisher` node subscribes to an image topic and a laser scan topic. It processes the received image data to detect colored markers (pink, yellow, green, blue) using OpenCV. The detected markers' positions are calculated in the camera frame and transformed into the map frame before publishing markers on the `visualization_marker_array` topic.

### Dependencies

- [rclpy](https://github.com/ros2/rclpy): ROS 2 Python client library
- [sensor_msgs](https://github.com/ros2/common_interfaces/tree/master/sensor_msgs): ROS standard messages for sensors
- [std_msgs](https://github.com/ros2/common_interfaces/tree/master/std_msgs): ROS standard messages
- [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge): ROS package for converting between ROS Image messages and OpenCV images
- [tf2_ros](https://github.com/ros2/geometry2/tree/ros2/tf2_ros): ROS package for transforming data between coordinate frames
- [visualization_msgs](https://github.com/ros2/common_interfaces/tree/master/visualization_msgs): ROS standard messages for visualization
- [message_filters](https://github.com/ros2/message_filters): ROS package for synchronizing message callbacks

### Usage

1. Install ROS 2 and the required dependencies.
2. Clone this repository into your ROS workspace.
3. Build the workspace using `colcon build`.
4. Run the ROS 2 system.
5. Launch the `ColorPublisher` node to start the color detection and marker publishing process.

## WallFollower Node

The `WallFollower` node implements a wall following behavior for a robot. It subscribes to laser scan and odometry topics to navigate the robot along a wall using distance measurements from the laser scan data. The robot adjusts its movement based on the detected distances to maintain a specified distance from the wall.

### Dependencies

- [rclcpp](https://github.com/ros2/rclcpp): ROS 2 C++ client library
- [geometry_msgs](https://github.com/ros2/common_interfaces/tree/master/geometry_msgs): ROS standard messages for geometric types
- [nav_msgs](https://github.com/ros2/common_interfaces/tree/master/nav_msgs): ROS standard messages for navigation

### Usage

1. Install ROS 2 and the required dependencies.
2. Clone this repository into your ROS workspace.
3. Build the workspace using `colcon build`.
4. Run the ROS 2 system.
5. Launch the `WallFollower` node to enable the robot's wall-following behavior.

## Simulation Recording
As requested by the Lecturer during the demonstration, the simulation recording can be found [here](https://youtu.be/yWMS20xZg8g)
## License

### WallFollower Node

The code in this repository is licensed under the Apache License, Version 2.0.

---

**Authors:** Taehun Lim (Darby), Ryan Shim, Claude Sammut (Modifications for COMP3431)

This code was originally developed by ROBOTIS CO., LTD., and modified for educational purposes.
