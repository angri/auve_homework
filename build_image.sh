#!/bin/sh
docker build \
    -t auve_homework:latest \
    --network=host \
    `dirname $0`
