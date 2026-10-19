import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    application_package = get_package_share_directory(
        'wheeltec_autorace_application'
    )

    lane_perception_config = os.path.join(
        application_package,
        'config',
        'lane_perception.yaml'
    )

    description_package = get_package_share_directory(
        'wheeltec_autorace_description'
    )

    models_path = os.path.join(
        description_package,
        'models'
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=(
            models_path
            + ':'
            + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
        )
    )

    gazebo_package = get_package_share_directory(
        'wheeltec_autorace_gazebo'
    )

    ros_gz_sim_package = get_package_share_directory(
        'ros_gz_sim'
    )

    world_path = os.path.join(
        gazebo_package,
        'worlds',
        'racetrack_world.sdf'
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

    # One controller process per launch. Never launch both and rely on enable flags.
    def selected_controller(context):
        selected = LaunchConfiguration('controller').perform(context)
        if selected == 'none':
            return []
        executable = {'pure_pursuit': 'pure_pursuit_controller',
                      'pid': 'pid_lane_controller'}[selected]
        return [Node(
            package='wheeltec_autorace_application', executable=executable,
            name=executable, output='screen',
            parameters=[os.path.join(application_package, 'config', executable + '.yaml'),
                        {'use_sim_time': True}],
        )]

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'], output='screen',
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

    def selected_perception(context):
        selected = LaunchConfiguration('controller').perform(context)
        # Existing CMD_V overlay must follow the controller actually launched.
        # No image-processing, target-selection or PP parameter changes here.
        speed_topic = ('/control/pid/current_speed' if selected == 'pid'
                       else '/control/pure_pursuit/current_speed')
        return [Node(
            package='wheeltec_autorace_application',
            executable='lane_perception', name='lane_perception', output='screen',
            parameters=[lane_perception_config,
                        {'use_sim_time': True, 'current_speed_topic': speed_topic,
                         'debug_controller': selected}],
        )]

    rqt_image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='lane_perception_image_view',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='pure_pursuit',
                              choices=['pure_pursuit', 'pid', 'none']),
        clock_bridge,
        OpaqueFunction(function=selected_controller),
        gazebo_resource_path,
        gazebo,
        cmd_vel_bridge,
        camera_image_bridge,
        camera_info_bridge,
        OpaqueFunction(function=selected_perception),
        rqt_image_view
    ])
