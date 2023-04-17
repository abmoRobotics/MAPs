#!/bin/bash
colcon build
source install/setup.sh
ros2 launch orchestrator orchestrator.launch.py
