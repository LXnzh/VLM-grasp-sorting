#!/bin/bash

# check if terminator is installed
if ! command -v terminator &> /dev/null; then
  echo "Terminator could not be found, install it with 'sudo apt install terminator'"
  exit 1
fi

# -----------------------------
# Define titles + commands here
# -----------------------------
TITLE1="ROS2: Mujoco Setup"
COMMAND1="ros2 launch ifl_air_ur_launch cell_small_ur_orbbec_robotiq_mujoco.launch.py launch_rviz:=false"

TITLE2="ROS2: MoveIt"
COMMAND2="ros2 launch ifl_air_ur_launch moveit_cell_small_ur_orbbec_robotiq.launch.py launch_rviz:=false"

TITLE3=""
COMMAND3=""

TITLE4=""
COMMAND4=""

# Define screen resolution and calculate window size (adjust these if necessary)
SCREEN_WIDTH=1920
SCREEN_HEIGHT=1080

# Calculate half screen dimensions for positioning
HALF_WIDTH=$((SCREEN_WIDTH / 2))
HALF_HEIGHT=$((SCREEN_HEIGHT / 2))

# Helper to launch a Terminator window with a custom title + command
run_term() {
  local title="$1"
  local geom="$2"
  local cmd="$3"

  # If cmd is empty, do nothing
  [ -z "$cmd" ] && return 0

  terminator -l default --geometry="$geom" \
    -e "bash -lc 'printf \"\033]0;%s\007\" \"$title\"; exec $cmd'" &
}

# Launch Terminator windows with specified titles/commands
run_term "$TITLE1" "800x340+0+0" "$COMMAND1"
sleep 3

run_term "$TITLE2" "800x340+$HALF_WIDTH+0" "$COMMAND2"
sleep 3

run_term "$TITLE3" "800x340+0+$HALF_HEIGHT" "$COMMAND3"
sleep 1

run_term "$TITLE4" "800x340+$HALF_WIDTH+$HALF_HEIGHT" "$COMMAND4"

# Wait for all processes to launch
wait
