# ROS2 Development Workspace

My personal ROS2 Humble development environment for learning and experimentation.

## Current Projects
- TurtleBot3 navigation and control
- (More to come...)

## Usage
Startup: 
```bash
docker-compose up -d
docker-compose exec ros2_dev bash
colcon build
```
Shutdown:
```bash
docker-compose down
```
If changes made to Dockerfile
```bash
docker-compose build
```