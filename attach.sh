#!/bin/bash

# Attach to running ros2_dev container
# docker-compose exec ros2_dev bash
docker-compose exec ros2_dev bash -c "source install/setup.bash && bash"