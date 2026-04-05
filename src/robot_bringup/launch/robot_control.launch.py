import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('robot_bringup'), 'urdf', 'robot.urdf.xacro'
        ])
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    controller_config = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'config', 'robot_controllers.yaml'
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen',
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
    )

    # Spawner for joint_state_broadcaster (start first)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawner for mecanum_drive_controller
    mecanum_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawner for arm_controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawner for imu_broadcaster
    imu_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delay controller spawners until controller_manager is started
    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        joint_state_broadcaster_spawner,
                        mecanum_drive_controller_spawner,
                        arm_controller_spawner,
                        imu_broadcaster_spawner,
                    ],
                ),
            ],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        delayed_spawners
    ])
