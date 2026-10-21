#!/bin/bash

# Run rviz
xhost +local:root

cd "$(dirname "$(realpath "${BASH_SOURCE[0]}")")/.."
docker run \
    -it \
    --rm \
    --privileged \
    --net=host \
    --ipc=host \
    --pid=host \
    --env DISPLAY=$DISPLAY \
    --volume ${HOME}/.Xauthority:/root/.Xauthority:rw \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "$(pwd)/overlay_ws/src:/overlay_ws/src:rw" \
    --name test \
    ros2-kortex \
    bash



