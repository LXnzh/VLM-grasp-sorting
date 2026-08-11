#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash

# Enable colcon autocomplete
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

alias cbp="colcon build --packages-select"

xtermcontrol --bg '#003'

