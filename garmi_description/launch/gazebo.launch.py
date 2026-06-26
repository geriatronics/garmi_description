# Copyright 2026 Technical University of Munich
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('garmi_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'garmi.urdf')
    
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    model_arg = DeclareLaunchArgument(name='model', default_value=default_model_path,
                                      description='Absolute path to robot urdf file')

    arm_controllers_arg = DeclareLaunchArgument(
        name='arm_controllers', default_value='trajectory',
        choices=['trajectory', 'velocity'],
        description="Which controllers to load for the arms: 'trajectory' "
                    "(JointTrajectoryController, GUI-friendly) or 'velocity' "
                    "(forward JointGroupVelocityController, for streaming/rosbags)")

    control_guis_arg = DeclareLaunchArgument(
        name='control_guis', default_value='true',
        choices=['true', 'false'],
        description='Launch the rqt GUIs for jogging the joints and steering '
                    'the base. Disable them when a node drives the robot.')

    use_trajectory = IfCondition(
        PythonExpression(["'", LaunchConfiguration('arm_controllers'), "' == 'trajectory'"]))
    use_velocity = IfCondition(
        PythonExpression(["'", LaunchConfiguration('arm_controllers'), "' == 'velocity'"]))

    gui_config = PathJoinSubstitution([get_package_share_directory('garmi_description'), 'config', 'gui.config'])

    # Start Gazebo sim with an empty world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        # launch_arguments={'gz_args': '-r empty.sdf -g ' + gui_config}.items()
        launch_arguments=[
            ('gz_args', ['-r ', 'empty.sdf',
                         ' --gui-config ',
                         gui_config])
        ]
    )

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    control_guis = IfCondition(LaunchConfiguration('control_guis'))

    rqt_node = Node(
        package='rqt_joint_trajectory_controller',
        executable='rqt_joint_trajectory_controller',
        parameters=[{'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', 'rcl.logging_rosout:=ERROR'],
        condition=control_guis,
    )

    rqt_steering_node = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        parameters=[{
            'default_topic': '/garmi_base_controller/reference',
            'default_stamped': True,
            'use_sim_time': True
        }],
        condition=control_guis,
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'garmi',
                   '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    # Delay ros2_control node spawning until the robot is fully loaded in Gazebo
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Arms: spawn either the trajectory or the velocity controllers,
    # depending on the 'arm_controllers' argument.
    load_arm_0_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['garmi_arm_0_controller'],
        condition=use_trajectory,
    )

    load_arm_1_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['garmi_arm_1_controller'],
        condition=use_trajectory,
    )

    load_arm_0_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['garmi_arm_0_velocity_controller'],
        condition=use_velocity,
    )

    load_arm_1_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['garmi_arm_1_velocity_controller'],
        condition=use_velocity,
    )

    load_head_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['garmi_head_controller'],
    )

    load_lift_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['garmi_lift_controller'],
    )

    load_base_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['garmi_base_controller'],
    )

    return LaunchDescription([
        model_arg,
        arm_controllers_arg,
        control_guis_arg,
        gazebo_launch,
        clock_bridge,
        rqt_node,
        rqt_steering_node,
        robot_state_publisher_node,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    load_joint_state_broadcaster,
                    load_arm_0_controller,
                    load_arm_1_controller,
                    load_arm_0_velocity_controller,
                    load_arm_1_velocity_controller,
                    load_head_controller,
                    load_lift_controller,
                    load_base_controller
                ],
            )
        ),
    ])
