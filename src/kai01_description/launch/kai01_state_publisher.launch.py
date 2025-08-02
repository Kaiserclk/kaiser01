import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,Command
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    urdf_file_name = 'kai01.urdf.xacro'

    use_gui = LaunchConfiguration('use_gui', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_joint_state=LaunchConfiguration('use_joint_state', default='true')

    use_gui_arg = DeclareLaunchArgument('use_gui', default_value=use_gui)
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value=use_rviz)
    use_sim_time_arg= DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    use_joint_state_arg=DeclareLaunchArgument('use_joint_state', default_value=use_joint_state)

    urdf = os.path.join(get_package_share_directory('kai01_description'),'urdf',urdf_file_name)
    rviz_config_file = os.path.join(get_package_share_directory('kai01_description'),'rviz','display.rviz')
    robot_description = Command(['xacro ', urdf])

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(use_joint_state),
        parameters=[{'source_list': ['/controller_manager/joint_states'],
                     'rate': 20.0}]
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui),
        remappings=[('/joint_states', 'joint_states')]
    )
  
    robot_state_publisher_ndoe=  Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        arguments=[urdf],
    )
    
    rviz_ndoe=  Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        output='screen')
    
    return LaunchDescription([
        use_gui_arg,
        use_rviz_arg,
        use_sim_time_arg,
        use_joint_state_arg,
        # joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_ndoe,
        rviz_ndoe,
    ])
