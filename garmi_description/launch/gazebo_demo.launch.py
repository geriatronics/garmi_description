# Copyright 2026 Technical University of Munich
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('garmi_description')

    # Reuse the standard Gazebo example (sim, robot, controllers), but load the
    # forward velocity controllers for the arms so the demo node can stream
    # joint velocities to them. The rqt control GUIs are disabled because the
    # demo node drives the whole robot itself.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'arm_controllers': 'velocity',
            'control_guis': 'false',
        }.items()
    )

    # The example controller node. Start it with a short delay so the
    # controllers have time to spawn and become active first.
    demo_node = Node(
        package='garmi_description',
        executable='demo_motion.py',
        name='garmi_demo_controller',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gazebo_launch,
        TimerAction(period=12.0, actions=[demo_node]),
    ])
