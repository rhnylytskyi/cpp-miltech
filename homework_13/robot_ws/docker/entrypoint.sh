#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /root/robot_ws/install/setup.bash

exec "$@"
