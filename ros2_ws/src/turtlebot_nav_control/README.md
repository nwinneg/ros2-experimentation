# Turtlebot Nav Control

Development workspace that uses the turtlebot3 burger platform to experiment with navigation and control implementations.

## Basic Usage
```bash
ros2 launch turtlebot_nav_control <launch_filename.launch.py>
```
* Waypoints are defined using the 2D Goal Pose topic from rviz
* nav2 slam toolbox used to maintain global and local mapping

## Nodes
- `simple_navigator` - Basic navigation controller that uses a PID to do waypoint navigation in an empty world
    - This is the first step used to test signal flow and initial implementation
    - There is no sophistocated sensing, perception, planning, or control outside of basic proportional yaw and distance control
- `map_visualizer` - Simple script node that generates a matplotlib visualization of the current map
    - Intended as a playground to understand how to interact with OccupancyGrid data structures
- `simple_planner` - Publishes A* planned path to /planned_path topic 
    - Implementation of A* path planning to define a trajectory between waypoints
- `simple_follower` - Basic standley style path follower
    - Initial motion control to follow a trajectory, model free for the first simple case (stanley style proportional control)

## Launch Files
- `empty_world_simple.launch.py` - Launches the empty world with just simple_navigator installed.
    - Intended to make sure things work at the most basic level.
- `empty_world.launch.py` - Launches simple navigator with slam toolbox running in mapping mode. 
    - The next step, now we can add blocks/obstacles to the world and watch as they are detected and added to the map.
- `test_simple_navigator.launch.py` - Launches in the turtlebot3_world instead of an empty one
    - No obstacle avoidance, just a straight path between current pose and goal pose
- `test_planner.launch.py` - Launches in turtlebot3_world with slam toolbox in navigation
    - No motion control at all, A* node plans a path from current pose to goal pose and publishes to /planned_path
- `test_path_follower.launch.py` - *In Progress*
    - Using planner node from previous to generate /planned_path and test follower node

## Additional Contents
- `test.launch.py` - Test file used to experiment with additional nav2 features