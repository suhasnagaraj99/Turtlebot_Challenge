# Turtlebot_Challenge

## ENPM 673: Perception for Autonomus Robots

## Instructions for running perception turtlebot3 challenge project

## Setup

Create a workpace

```sh
mkdir -p project_ws/src
cd ~/project_ws/src
```
Place the group8 package inside the src folder

Source ROS environment(Enable ROS commands)

```sh
source /opt/ros/humble/setup.bash
```

Build the workspace

```sh
cd ~\project_ws
colcon build
```

Source ROS (Package will be identified)

```sh
source install/setup.bash
```

## Launch the Gazebo Simulation

SSH into the turtlebot3 and run the following two commands

```sh
ros2 launch turtlebot3_project3 competition_world.launch.py
```

## Run the nodes to start moving the turtlebot

Run the following commands in separate terminals

```sh
ros2 run group8 detect_horizon
ros2 run group8 paper_centroid
ros2 run group8 optical
ros2 run group8 yolo
```

To visualuize the output from the nodes, run the following command and select the topic as 'camera_feed'

```sh
ros2 run rqt_image_view rqt_image_view
```

Finally run the master node

```sh
ros2 run group8 turtlebot_controller
```
