#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_share = get_package_share_directory(
        "wheeltec_autorace_description"
    )
    gazebo_share = get_package_share_directory(
        "wheeltec_autorace_gazebo"
    )
    bringup_share = get_package_share_directory(
        "wheeltec_autorace_bringup"
    )

    generator_default = os.path.expanduser(
        "~/wheeltec_autorace_ws/src/"
        "wheeltec_autorace_sim/tools/generate_multicorner_track.py"
    )

    generator_script = LaunchConfiguration("generator_script")
    min_track_size = LaunchConfiguration("min_track_size")
    max_track_size = LaunchConfiguration("max_track_size")
    min_turns = LaunchConfiguration("min_turns")
    max_turns = LaunchConfiguration("max_turns")
    variation = LaunchConfiguration("variation")

    course_png = os.path.join(
        description_share,
        "models",
        "racetrack",
        "materials",
        "textures",
        "course.png",
    )

    model_sdf = os.path.join(
        description_share,
        "models",
        "racetrack",
        "model.sdf",
    )

    world_sdf = os.path.join(
        gazebo_share,
        "worlds",
        "racetrack_world.sdf",
    )

    debug_png = "/tmp/wheeltec_random_track_debug.png"
    metadata_json = "/tmp/wheeltec_random_track.json"

    prepare_track = ExecuteProcess(
        cmd=[
            "python3",
            generator_script,
            "--random",
            "--min-track-size",
            min_track_size,
            "--max-track-size",
            max_track_size,
            "--min-random-turns",
            min_turns,
            "--max-random-turns",
            max_turns,
            "--variation",
            variation,
            "--output",
            course_png,
            "--debug-output",
            debug_png,
            "--model-sdf",
            model_sdf,
            "--world-sdf",
            world_sdf,
            "--metadata-output",
            metadata_json,
        ],
        output="screen",
    )

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share,
                "launch",
                "simulation.launch.py",
            )
        )
    )

    def on_track_generator_exit(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "[ERROR] Random track generation failed. "
                        "Gazebo will NOT be started."
                    )
                ),
                EmitEvent(
                    event=Shutdown(
                        reason="Random track generation failed."
                    )
                ),
            ]

        return [
            LogInfo(
                msg=(
                    "Random racetrack ready. "
                    f"Metadata: {metadata_json} | Debug: {debug_png}"
                )
            ),
            simulation_launch,
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "generator_script",
                default_value=generator_default,
                description=(
                    "Path to generate_multicorner_track.py. "
                    "The default matches this project's workspace layout."
                ),
            ),
            DeclareLaunchArgument(
                "min_track_size",
                default_value="9.0",
                description="Minimum random track size in meters.",
            ),
            DeclareLaunchArgument(
                "max_track_size",
                default_value="15.0",
                description="Maximum random track size in meters.",
            ),
            DeclareLaunchArgument(
                "min_turns",
                default_value="6",
                description=(
                    "Minimum requested random corner count. "
                    "Closed orthogonal tracks use an even actual count."
                ),
            ),
            DeclareLaunchArgument(
                "max_turns",
                default_value="20",
                description="Maximum requested random corner count.",
            ),
            DeclareLaunchArgument(
                "variation",
                default_value="0.65",
                description="Random track shape variation, 0.0-1.0.",
            ),
            LogInfo(
                msg=(
                    "Generating a fresh random racetrack before Gazebo startup..."
                )
            ),
            prepare_track,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=prepare_track,
                    on_exit=on_track_generator_exit,
                )
            ),
        ]
    )
