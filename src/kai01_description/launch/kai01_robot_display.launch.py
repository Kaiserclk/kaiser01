import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,Command
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    urdf_file_name = 'kai01.urdf.xacro'
    pkg_dir=get_package_share_directory('kai01_description')
    urdf = os.path.join(pkg_dir,'urdf',urdf_file_name)
    rviz_config_file = os.path.join(pkg_dir,'rviz','display.rviz')
    robot_description = Command(['xacro ', urdf])

    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        remappings=[('/joint_states', 'joint_states')]
    )
  
    robot_state_publisher_ndoe=  Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        arguments=[urdf],
    )
    
    rviz_ndoe=  Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')
    
    return LaunchDescription([
        joint_state_publisher_gui_node,
        robot_state_publisher_ndoe,
        rviz_ndoe,
    ])
