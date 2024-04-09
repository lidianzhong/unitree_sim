import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Package Directories
    pkg_go2_description = get_package_share_directory('go2_description')
    
    # Parse robot description from xacro
    robot_description_file =  os.path.join(pkg_go2_description, 'xacro', 'robot.xacro')
    robot_desc = xacro.process_file(robot_description_file).toxml()

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # Controllers
    controllers = os.path.join(pkg_go2_description, 'config', 'go2_controllers.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'go2'],
                    output='screen')


    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_desc}, controllers],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        node_controller_manager
    ])

