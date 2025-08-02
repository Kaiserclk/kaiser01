import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    use_sim_time_arg= DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    x_pose_arg= DeclareLaunchArgument('x_pose', default_value=x_pose)
    y_pose_arg= DeclareLaunchArgument('y_pose', default_value=y_pose)

    world = os.path.join(get_package_share_directory('kai01_gazebo_simulation'),'worlds','turtlebot3_dqn_stage3.world')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments=[('world', world),('verbose','true')]
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('kai01_description'), 'launch', 'kai01_state_publisher.launch.py')
        )
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        # launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'kai01_robot',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )
    
    return LaunchDescription([
        x_pose_arg,
        y_pose_arg,
        use_sim_time_arg,
        robot_state_publisher_cmd,
        # gzserver_cmd,
        # gzclient_cmd,
        start_gazebo_ros_spawner_cmd,

    ])
