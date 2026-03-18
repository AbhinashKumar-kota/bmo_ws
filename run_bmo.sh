#!/bin/bash
# -----------------------------------------------------------------------
# BMO Simulation Launcher
#
# Starts the full BMO swarm simulation in a tmux session with 3 panes:
#   Pane 0: Gazebo + ROS bridge (bmo_sim)
#   Pane 1: BMO coordinator (bmo_control)
#   Pane 2: Topic monitor (for debugging)
#
# Usage:
#   ./run_bmo.sh                                          # default world, info logging
#   ./run_bmo.sh debug                                    # default world, debug logging
#   ./run_bmo.sh info center_source_world.sdf             # center lamp world
#
# To stop: tmux kill-session -t bmo
# To attach: tmux attach -t bmo
# -----------------------------------------------------------------------

SESSION="bmo"
LOG_LEVEL="${1:-info}"
WORLD="${2:-single_source_world.sdf}"

# Kill existing session if any
tmux kill-session -t $SESSION 2>/dev/null

# Common setup commands
SETUP="source /opt/ros/humble/setup.bash && source ~/bmo_ws/install/setup.bash"
EXPORTS="export GZ_SIM_RESOURCE_PATH=\"\$HOME/bmo_ws/install/bmo_sim/share/bmo_sim/models:\$HOME/cf_ws/simulation_ws/crazyflie-simulation/simulator_files/gazebo/\""

# Create tmux session with first pane: Gazebo + bridge
tmux new-session -d -s $SESSION -n main

# Pane 0: Gazebo simulation + ROS bridge
tmux send-keys -t $SESSION:main.0 "$SETUP && $EXPORTS && ros2 launch bmo_sim bmo_simulation.launch.py world:=$WORLD" Enter

# Wait for Gazebo and bridge to start before launching coordinator
sleep 8

# Split horizontally: Pane 1 — BMO coordinator
tmux split-window -h -t $SESSION:main
tmux send-keys -t $SESSION:main.1 "$SETUP && ros2 launch bmo_control bmo_control.launch.py log_level:=$LOG_LEVEL" Enter

# Split pane 1 vertically: Pane 2 — Topic monitor
tmux split-window -v -t $SESSION:main.1
tmux send-keys -t $SESSION:main.2 "$SETUP && sleep 5 && echo '=== Checking topics ===' && ros2 topic list | grep cf && echo '=== cf0 intensity ===' && ros2 topic echo /cf0/light_intensity --once && echo '=== cf0 odom ===' && ros2 topic echo /cf0/odom --once" Enter

# Attach to the session
tmux attach -t $SESSION
