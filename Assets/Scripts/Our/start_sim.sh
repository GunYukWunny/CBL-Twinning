#!/bin/bash

# Set ROS distro manually if not set
ROS_DISTRO=${ROS_DISTRO:-foxy}

WORKSPACE="$HOME/example_ws"

cd "$WORKSPACE" || { echo "❌ Workspace not found!"; exit 1; }

# Build workspace once before launching anything
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
colcon build
source install/setup.bash

export TURTLEBOT3_MODEL=burger

# Function to open a new gnome-terminal and run a command
run_in_new_terminal() {
    gnome-terminal -- bash -c "$1; exec bash"
}

echo "Launching Gazebo simulation..."
run_in_new_terminal "source /opt/ros/$ROS_DISTRO/setup.bash && source $WORKSPACE/install/setup.bash && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"

echo "Launching ROS-Unity connection..."
run_in_new_terminal "source /opt/ros/$ROS_DISTRO/setup.bash && source $WORKSPACE/install/setup.bash && ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1"

echo "Launching Cartographer..."
run_in_new_terminal "source /opt/ros/$ROS_DISTRO/setup.bash && source $WORKSPACE/install/setup.bash && ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True"

echo "Please close the Cartographer RViz window manually to prevent crashes."

echo "Launching Map Sharing Server..."
run_in_new_terminal "source /opt/ros/$ROS_DISTRO/setup.bash && source $WORKSPACE/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f $HOME/map"

echo "Launching Navigation..."
run_in_new_terminal "source /opt/ros/$ROS_DISTRO/setup.bash && source $WORKSPACE/install/setup.bash && ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml"

echo "All nodes launched. Now launch Unity and press play."

