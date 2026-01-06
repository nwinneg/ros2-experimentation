# ROS2 Development Workspace

My personal ROS2 Humble development environment for learning and experimentation.

## Current Projects
- Differential drive navigation and control (Turtlebot3 & Nav2)
- 6-Axis arm manipulation (ur5 & MoveIt)

## Usage
Startup: 
```bash
docker-compose up -d
docker-compose exec ros2_dev bash
colcon build --symlink-install
```
Shutdown:
```bash
docker-compose down
```
If changes made to Dockerfile
```bash
docker-compose build
```