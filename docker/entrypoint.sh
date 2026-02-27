#!/usr/bin/env bash
# Run as interactive shell so .bashrc is sourced (matches manual workflow exactly)
exec bash -ic '
cd /ros2_ws &&
./cbuild &&
source setup_uri.sh eno2 &&
source install/setup.bash &&
ros2 launch g1pilot bringup_launcher.launch.py enable_collision_avoidance:=${ENABLE_COLLISION_AVOIDANCE:-false}
'
#unset RMW_IMPLEMENTATION &&
