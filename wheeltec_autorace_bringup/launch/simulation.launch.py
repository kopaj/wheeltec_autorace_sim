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

    camera_image_gz_topic = (
        '/world/wheeltec_autorace'
        '/model/roboworks'
        '/link/camera_link'
        '/sensor/my_rgbd_camera'
        '/image'
    )

    camera_info_gz_topic = (
        '/world/wheeltec_autorace'
        '/model/roboworks'
        '/link/camera_link'
        '/sensor/my_rgbd_camera'
        '/camera_info'
    )

    camera_image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_image_bridge',
        output='screen',
        arguments=[
            camera_image_gz_topic
            + '@sensor_msgs/msg/Image'
            + '[ignition.msgs.Image'
        ],
        remappings=[
            (
                camera_image_gz_topic,
                '/camera/image_raw'
            )
        ]
    )

    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_info_bridge',
        output='screen',
        arguments=[
            camera_info_gz_topic
            + '@sensor_msgs/msg/CameraInfo'
            + '[ignition.msgs.CameraInfo'
        ],
        remappings=[
            (
                camera_info_gz_topic,
                '/camera/camera_info'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        cmd_vel_bridge,
        camera_image_bridge,
        camera_info_bridge
    ])
