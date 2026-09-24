import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    gazebo_package = get_package_share_directory(
        'wheeltec_autorace_gazebo'
    )

    ros_gz_sim_package = get_package_share_directory(
        'ros_gz_sim'
    )

    world_path = os.path.join(
        gazebo_package,
        'worlds',
        'wheeltec_world.sdf'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_package,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 {world_path}'
        }.items()
    )

    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        arguments=[
            '/roboworks/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
        ],
        remappings=[
            ('/roboworks/cmd_vel', '/cmd_vel')
        ]
    )

    return LaunchDescription([
        gazebo,
        cmd_vel_bridge
    ])
