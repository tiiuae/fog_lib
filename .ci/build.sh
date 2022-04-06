#!/bin/bash
set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

ROS_DISTRO="galactic"

# get the path to this script
MY_PATH=`pwd`

echo "Starting build"

mkdir -p ~/ros2_workspace/src
cd ~/ros2_workspace/src
ln -s "$MY_PATH" 

source /opt/ros/$ROS_DISTRO/setup.bash

cd ~/ros2_workspace
command colcon build

echo "build ended"
