# Autonomous Navigation Robot - ROS2

A ROS2 project implementing autonomous navigation with obstacle avoidance using TurtleBot3 simulation.

## Demo Video
[![Demo Video](https://img.youtube.com/vi/NocbXcP8jg0/0.jpg)](https://www.youtube.com/watch?v=NocbXcP8jg0)

*Click the image above to watch the robot in action!*

## Features
- Real-time obstacle detection using laser scanner
- Autonomous navigation with dynamic obstacle avoidance
- Built with ROS2 Jazzy

## Setup
```bash
# Install dependencies
sudo apt install ros-jazzy-turtlebot3* -y

# Build the workspace
cd ros2_ws
colcon build
source install/setup.bash
```

## Running
Terminal 1 - Start simulation:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Terminal 2 - Run navigator:
```bash
ros2 run autonomous_navigator navigator
```

## Status
1. Basic obstacle avoidance working
2. Waypoint navigation (coming next)
