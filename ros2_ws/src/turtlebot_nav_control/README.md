# Turtlebot Nav Control

Development workspace that uses the turtlebot3 burger platform to experiment with navigation and control implementations.

## Basic Usage
```bash
ros2 launch turtlebot_nav_control empty_world.launch.py
```
* Waypoints are defined using the 2D Goal Pose topic from rviz
* nav2 slam toolbox used to maintain global and local mapping

## Nodes
- `simple_navigator` - Basic navigation controller that uses a PID to do waypoint navigation in an empty world
    - This is the first step used to test signal flow and initial implementation
    - There is no sophistocated sensing, perception, planning, or control outside of basic proportional yaw and distance control
- `map_visualizer` - Simple script node that generates a matplotlib visualization of the current map
    - Intended as a playground to understand how to interact with OccupancyGrid data structures
- `a-star-planner` - *IN PROGRESS*
    - Implementation of A* path planning to define a trajectory between waypoints

## Launch Files
- `empty_world_simple.launch.py` - Launches the empty world with just simple_navigator installed.
    - Intended to make sure things work at the most basic level.
- `empty_world.launch.py` - Launches simple navigator with slam toolbox running in mapping mode. 
    - The next step, now we can add blocks/obstacles to the world and watch as they are detected and added to the map.