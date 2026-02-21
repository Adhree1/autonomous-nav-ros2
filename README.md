# Autonomous Navigation Robot - ROS2

A ROS2 project implementing autonomous navigation with waypoint following and obstacle avoidance using TurtleBot3 simulation.

## Demo Videos

**Waypoint Navigation with RViz Visualization:**
[![Waypoint Navigation Demo](https://img.youtube.com/vi/V0O-TULP-uc/0.jpg)](https://www.youtube.com/watch?v=V0O-TULP-uc)

**Basic Obstacle Avoidance (Initial Version):**
[![Basic Demo](https://img.youtube.com/vi/NocbXcP8jg0/0.jpg)](https://www.youtube.com/watch?v=NocbXcP8jg0)

*Watch the robot navigate through multiple waypoints while avoiding obstacles in real-time!*

## Features
- ✅ **Waypoint Navigation** - Robot follows predefined coordinates (x, y positions)
- ✅ **Real-time Obstacle Avoidance** - Uses laser scanner for collision detection
- ✅ **Sensor Fusion** - Combines odometry and LIDAR data
- ✅ **Proportional Control** - Smooth turning based on angle error
- ✅ **Priority-Based Decision Making** - Safety-first navigation logic
- ✅ **Multi-Goal Mission Execution** - Completes full path and returns to start

## Technical Implementation

### Navigation Algorithm
The robot uses a priority-based finite state machine:
1. **Priority 1:** Obstacle avoidance (safety first)
2. **Priority 2:** Turn toward goal if misaligned
3. **Priority 3:** Drive forward to waypoint

### Key Concepts
- **Distance Calculation:** Euclidean distance using Pythagorean theorem
- **Angle Calculation:** `atan2()` for heading computation
- **Proportional Control:** Turn rate proportional to angle error
- **Goal Detection:** Waypoint considered reached within 0.3m threshold

## Setup

### Prerequisites
- Ubuntu 24.04
- ROS2 Jazzy
- TurtleBot3 packages

### Installation
```bash
# Install dependencies
sudo apt update
sudo apt install ros-jazzy-turtlebot3* -y

# Clone the repository
git clone https://github.com/Adhree1/autonomous-nav-ros2.git
cd autonomous-nav-ros2

# Build the workspace
cd ros2_ws
colcon build
source install/setup.bash
```

## Running the Robot

### Terminal 1 - Start Simulation
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 2 - Run Navigator
```bash
cd ~/autonomous-nav-ros2/ros2_ws
source install/setup.bash
ros2 run autonomous_navigator navigator
```

The robot will automatically navigate through 4 waypoints:
- Waypoint 1: (2.0, 0.0) - 2 meters forward
- Waypoint 2: (2.0, 2.0) - 2 meters left
- Waypoint 3: (0.0, 2.0) - Return along x-axis
- Waypoint 4: (0.0, 0.0) - Return to start

## Project Structure
```
autonomous-nav-ros2/
├── ros2_ws/
│   └── src/
│       └── autonomous_navigator/
│           ├── autonomous_navigator/
│           │   └── navigator.py          # Main navigation node
│           ├── package.xml
│           └── setup.py
└── README.md
```

## How It Works

### Sensor Inputs
- **Laser Scanner (`/scan`):** 360° obstacle detection
- **Odometry (`/odom`):** Robot position (x, y, yaw)

### Control Output
- **Velocity Commands (`/cmd_vel`):** Linear and angular velocity

### Navigation Logic
```python
if obstacle_detected:
    turn_to_avoid()
elif not_facing_goal:
    turn_toward_goal()
else:
    drive_to_goal()
```

## Future Enhancements
🚧 Dynamic waypoint configuration via parameters  
🚧 Launch file for single-command startup  
🚧 Path visualization in RViz  
🚧 Advanced path planning (A*, RRT)  
🚧 Dynamic obstacle tracking  

## Skills Demonstrated
- ROS2 node development in Python
- Sensor data processing (LIDAR, odometry)
- Control algorithms (proportional control)
- State machine design
- Coordinate geometry and trigonometry
- Git version control

## License
MIT License - feel free to use for learning!

## Author
Built as a learning project to demonstrate ROS2 navigation concepts.
