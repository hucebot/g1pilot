# Copyright Ioannis Tsikelis
#
# Script to set up the network interface for CycloneDDS

#!/bin/bash
if [ $# -eq 0 ]
then
    echo "Usage: source setup.sh [network interface (or lo)]"
    return 1
fi

echo "Sourcing installed packages"
source /opt/ros/${ROS_DISTRO}/setup.bash
source ./install/setup.bash

# For autocompletion to work in terminal
# eval "$(register-python-argcomplete3 ros2)"
# eval "$(register-python-argcomplete3 colcon)"



echo "Setting up DDS"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General>
                        <Interfaces>
                        <NetworkInterface name="'$1'" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

if [ "$1" = "lo"  ]
then
    # Need to enable multicast if using localhost
    echo "Enabling multicast"
    ip link set lo multicast on
fi

echo "Setting G1_INTERFACE to $1"
export G1_INTERFACE=$1

echo "Done, let's try!"
