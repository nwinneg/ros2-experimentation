# Turtlebot Nav Control

Development workspace that uses the turtlebot3 burger platform to experiment with navigation and control implementations.

## Basic Usage
```bash
ros2 launch turtlebot_nav_control empty_world.launch.py
```
* Waypoints are defined using the 2D Goal Pose topic from rviz

## Nodes
- `simple_navigator` - Basic navigation controller that uses a PID to do waypoint navigation in an empty world
    - This is the first step used to test signal flow and initial implementation
    - There is no sophistocated sensing, perception, planning, or control outside of basic proportional yaw and distance control