from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory(
        "wheeltec_autorace_application"
    )

    config = os.path.join(
        package_share,
        "config",
        "pure_pursuit_controller.yaml",
    )

    return LaunchDescription([
        Node(
            package="wheeltec_autorace_application",
            executable="pure_pursuit_controller",
            name="pure_pursuit_controller",
            output="screen",
            parameters=[config],
        )
    ])
