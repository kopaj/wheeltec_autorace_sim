"""Starts only observation; existing simulation/controllers must already be running."""
import json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    ref_path = str(Path(LaunchConfiguration('reference').perform(context)).expanduser().resolve())
    data = json.loads(Path(ref_path).read_text())
    args = ['--reference', ref_path,
            '--label', LaunchConfiguration('label').perform(context),
            '--output', LaunchConfiguration('output').perform(context),
            '--duration', LaunchConfiguration('duration').perform(context)]
    config = LaunchConfiguration('controller_config').perform(context)
    if config:
        args += ['--controller-config', config]
    return [
        Node(package='wheeltec_autorace_evaluation', executable='ground_truth_bridge',
             parameters=[{'world_name': data['world_name'], 'model_name': data['model_name']}],
             output='screen'),
        Node(package='wheeltec_autorace_evaluation', executable='evaluate_tracking',
             arguments=args, output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('reference', description='Verified reference.json absolute path'),
        DeclareLaunchArgument('label', default_value='pid_trial01'),
        DeclareLaunchArgument('output', default_value=str(Path.home()/'wheeltec_results')),
        DeclareLaunchArgument('duration', default_value='60.0'),
        DeclareLaunchArgument('controller_config', default_value=''),
        OpaqueFunction(function=setup),
    ])
