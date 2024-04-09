#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/foxy/setup.bash
source $HOME/go2_ws/cyclonedds_ws/install/setup.bash
source $HOME/go2_ws/Go2_ROS2_example/install/setup.bash
source $HOME/go2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp2s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
