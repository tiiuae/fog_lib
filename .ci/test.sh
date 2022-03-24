#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

ROS_DISTRO="galactic"

echo "Starting test"

cd ~/ros2_workspace
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_workspace/install/setup.bash

export RCUTILS_COLORIZED_OUTPUT=1;
export ROS_LOCALHOST_ONLY=1

PACKAGE="fog_lib"
VERBOSE=0

[ "$VERBOSE" = "0" ] && TEXT_OUTPUT=""
[ "$VERBOSE" = "1" ] && TEXT_OUTPUT="-t"

colcon test --packages-select $PACKAGE # it has to be fully build normally before building with --catkin-make-args tests
colcon test-result --all

echo "Test ended"
