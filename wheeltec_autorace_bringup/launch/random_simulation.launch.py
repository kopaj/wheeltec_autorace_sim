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
    # Fail before changing the track if the exporter package is unavailable.
    get_package_share_directory("wheeltec_autorace_evaluation")

    generator_default = os.path.expanduser(
        "~/wheeltec_pp_clean_ws/src/"
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

    debug_png = LaunchConfiguration("debug_file")
    metadata_json = LaunchConfiguration("metadata_file")
    reference_json = LaunchConfiguration("reference_file")

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
        ),
        launch_arguments={"controller": LaunchConfiguration("controller")}.items(),
    )

    prepare_reference = ExecuteProcess(
        cmd=[
            "ros2", "run", "wheeltec_autorace_evaluation", "export_reference",
            "--metadata", metadata_json,
            "--generator", generator_script,
            "--variation", variation,
            "--texture", course_png,
            "--model", model_sdf,
            "--robot-model", os.path.join(description_share, "models", "roboworks", "model.sdf"),
            "--world", world_sdf,
            "--output", reference_json,
            "--overwrite",
        ],
        output="screen",
    )

    def on_reference_exit(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg="[ERROR] Reference export failed. Gazebo will NOT be started."),
                EmitEvent(event=Shutdown(reason="Reference export failed.")),
            ]
        return [
            LogInfo(msg=["Verified reference ready: ", reference_json]),
            simulation_launch,
        ]

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
                msg=["Random racetrack ready. Metadata: ", metadata_json,
                     ". Exporting matching reference..."]
            ),
            prepare_reference,
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("controller", default_value="pure_pursuit",
                                  choices=["pure_pursuit", "pid", "none"]),
            DeclareLaunchArgument(
                "reference_file",
                default_value=os.path.expanduser("~/wheeltec_benchmark/reference.json"),
                description="Current reference; replaced after every successful generation/export.",
            ),
            DeclareLaunchArgument(
                "metadata_file",
                default_value=os.path.expanduser("~/wheeltec_benchmark/track_metadata.json"),
                description="Persistent generation metadata for the current track.",
            ),
            DeclareLaunchArgument(
                "debug_file",
                default_value=os.path.expanduser("~/wheeltec_benchmark/track_debug.png"),
                description="Debug image of the current generated track.",
            ),
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
            # Register both exit handlers before starting either process.
            RegisterEventHandler(
                OnProcessExit(
                    target_action=prepare_track,
                    on_exit=on_track_generator_exit,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=prepare_reference, on_exit=on_reference_exit)
            ),
            prepare_track,
        ]
    )
