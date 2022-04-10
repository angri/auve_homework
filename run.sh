#!/bin/sh
nvidia-docker run \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume $XAUTHORITY:/tmp/.Xauthority \
    --device=/dev/dri:/dev/dri \
    -e DISPLAY=$DISPLAY \
    --network=host \
    --rm \
    -it \
    --gpus all \
    auve_homework:latest
