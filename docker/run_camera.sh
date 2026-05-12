#!/usr/bin/env bash
# No display needed on the robot — camera container only publishes ROS 2 topics.

docker run \
        -it \
        --net host \
        --privileged \
        --device-cgroup-rule='c 81:* rmw' \
        -v /dev:/dev \
        -v "$(pwd)/../":/ros2_ws/src/g1pilot \
        -v "$(pwd)/../config/livox_mid.json":/ros2_ws/src/livox_ros_driver2/config/MID360_config.json \
        -w /ros2_ws \
        --group-add video \
        g1pilot_camera:latest
