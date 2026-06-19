from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 启动参数
    use_camera = LaunchConfiguration('use_camera')
    use_radia = LaunchConfiguration('use_radia')

    # 雷达节点
    g4_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[{
            'port': '/dev/ydlidar',
            'frame_id': 'lidar',
            'ignore_array': '',
            'baudrate': 230400,
            'lidar_type': 1,
            'device_type': 0, #串口设备
            'sample_rate': 9,
            'intensity_bit': 0,
            'abnormal_check_count': 4, #异常检测
            'fixed_resolution': True,
            'auto_reconnect': True, #自动重连
            'reversion': True,  #旋转 180°，用于调整雷达安装方向
            'inverted': True,   #逆时针扫描，与 reversion 配合调整坐标系
            'isSingleChannel': False,
            'intensity': False,
            'support_motor_dtr': False,
            'invalid_range_is_inf': False,
            'angle_min': -180.0,
            'angle_max': 180.0,
            'range_min': 0.05,
            'range_max': 16.0,
            'frequency': 12.0
        }],
        condition=IfCondition(use_radia)
    )

    # 相机启动
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'launch',
                'dabai_dcw.launch.py'
            ])
        ),
        condition=IfCondition(use_camera)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_camera', default_value='False',
                              description='是否启动相机'),
        DeclareLaunchArgument('use_radia', default_value='True',
                              description='是否启动雷达'),
        g4_node,
        camera_launch
    ])
