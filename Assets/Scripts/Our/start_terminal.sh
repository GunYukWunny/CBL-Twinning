#!/bin/bash
source install/setup.bash && colcon build && source install/setup.bash && export TURTLEBOT3_MODEL=burger
exec bash
