#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/opt/catkin_ws/devel/setup.bash"

roslaunch traffic_light_fetcher test.launch
