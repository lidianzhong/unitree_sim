#!/bin/bash
echo "Setup unitree ros2 simulation environment"
source /opt/ros/foxy/setup.bash
source $HOME/go2_ws/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
