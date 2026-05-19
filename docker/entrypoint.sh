#!/usr/bin/env bash
# Run as interactive shell so .bashrc is sourced (matches manual workflow exactly)
if [ -z "$G1_INTERFACE" ]; then
    echo "ERROR: G1_INTERFACE environment variable is not set."
    echo "Set it to your network interface, e.g.: G1_INTERFACE=eno2"
    exit 1
fi

bash /sdk_setup.sh

exec bash -ic '
cd /ros2_ws &&
./cbuild &&
source setup_uri.sh ${G1_INTERFACE} &&
source install/setup.bash &&
ros2 launch g1pilot bringup_launcher.launch.py enable_collision_avoidance:=${ENABLE_COLLISION_AVOIDANCE:-false}
'
#unset RMW_IMPLEMENTATION &&
