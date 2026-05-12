# For graphics
xhost +local:docker

docker run \
        -it \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=0" \
        --env="ROS_DOMAIN_ID=1" \
        --net host \
        --privileged \
        --device-cgroup-rule='c 81:* rmw' \
        -v /dev:/dev \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -v `pwd`/../:/ros2_ws/src/g1pilot \
        -v `pwd`/../config/livox_mid.json:/ros2_ws/src/livox_ros_driver2/config/MID360_config.json  \
        -v $(pwd)/setup_uri.sh:/ros2_ws/setup_uri.sh \
        -v $(pwd)/cbuild:/ros2_ws/cbuild \
        -w /ros2_ws \
        --group-add video \
        g1pilot:latest