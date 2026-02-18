#!/usr/bin/env bash
# Run as interactive shell so .bashrc is sourced (matches manual workflow exactly)
exec bash -ic '
cd /ros2_ws &&
./cbuild &&
unset RMW_IMPLEMENTATION &&
source install/setup.bash &&
ros2 launch g1pilot bringup_launcher.launch.py
'
# source setup_uri.sh eno2 &&
