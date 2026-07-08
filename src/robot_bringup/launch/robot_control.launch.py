import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ArmControllerType",
            default_value='arm_controller',
            description="控制器类型arm_controller:常规位置控制 moveit_controller:MoveIt控制",
        )
    )
    ArmControllerType = LaunchConfiguration('ArmControllerType')
    
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('kai01_description'), 'urdf', 'kai01.urdf.xacro'
        ])
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    controller_config = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'config', 'robot_controllers.yaml'
    ])

    arm_controller_config = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'config', 'arm_controller.yaml'
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
        parameters=[controller_config, arm_controller_config],
        output='screen',
        remappings=[
            ('~/robot_description', '/robot_description'),
            # mecanum_drive_controller 话题重映射
            ('/mecanum_drive_controller/cmd_vel', '/cmd_vel'),
            ('/mecanum_drive_controller/odometry', '/odom'),
            ('/mecanum_drive_controller/tf_odometry', '/tf'),
            ('/imu_broadcaster/imu', '/imu/data_raw')
        ],
    )

    imu_filter_madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('robot_bringup'), 'config', 'common.yaml'
        ])],
    )
    # Spawner for joint_state_broadcaster (start first)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0'
        ],
        output='screen',
    )

    # Spawner for mecanum_drive_controller
    mecanum_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mecanum_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0'
        ],
        output='screen',
    )

    # Spawner for arm_position_controller
    arm_position_controller_spawner = Node(
        condition=IfCondition(EqualsSubstitution(ArmControllerType, 'arm_controller')),
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_position_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0'
        ],
        output='screen',
    )
    
    # Spawner for arm_controller
    arm_controller_spawner = Node(
        condition=IfCondition(EqualsSubstitution(ArmControllerType, 'moveit_controller')),
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0'
        ],
        output='screen',
    )
    hand_controller_spawner = Node(
        condition=IfCondition(EqualsSubstitution(ArmControllerType, 'moveit_controller')),
        package='controller_manager',
        executable='spawner',
        arguments=[
            'hand_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0'
        ],
        output='screen',
    )

    # Spawner for imu_broadcaster
    imu_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'imu_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0'
        ],
        output='screen',
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('robot_bringup'), 'config', 'ekf.yaml'
        ])],
        remappings=[('odometry/filtered', '/odom')],
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
                        arm_position_controller_spawner,
                        arm_controller_spawner,
                        # hand_controller_spawner,
                        imu_broadcaster_spawner,
                    ],
                ),
            ],
        )
    )

    return LaunchDescription([
        *declared_arguments,
        robot_state_publisher_node,
        controller_manager_node,
        delayed_spawners,
        # imu_filter_madgwick_node,
        # robot_localization_node

    ])
