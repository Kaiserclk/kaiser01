from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare arguments
    args = [
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('depth_registration', default_value='true'),  #启用深度帧与彩色帧的对齐。
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('usb_port', default_value=''),
        DeclareLaunchArgument('device_num', default_value='1'),
        DeclareLaunchArgument('vendor_id', default_value='0x2bc5'),
        DeclareLaunchArgument('product_id', default_value=''),
        DeclareLaunchArgument('enable_point_cloud', default_value='true'),  #是否启用点云
        DeclareLaunchArgument('enable_colored_point_cloud', default_value='false'), # 是否启用彩色点云
        DeclareLaunchArgument('cloud_frame_id', default_value=''),
        DeclareLaunchArgument('point_cloud_qos', default_value='default'),
        DeclareLaunchArgument('connection_delay', default_value='100'), #重新打开设备的延迟时间（毫秒）。
        DeclareLaunchArgument('color_width', default_value='640'), #彩色图像的宽度（像素）。
        DeclareLaunchArgument('color_height', default_value='480'), #彩色图像的高度（像素）。
        DeclareLaunchArgument('color_fps', default_value='30'), #彩色图像的帧率（fps）。
        DeclareLaunchArgument('color_format', default_value='MJPG'), #彩色图像的格式，可选值有：'MJPG', 'YUYV', 'RGB8', 'BGR8'。
        DeclareLaunchArgument('enable_color', default_value='true'), #是否启用彩色图像。
        DeclareLaunchArgument('flip_color', default_value='false'), #是否翻转彩色图像。
        DeclareLaunchArgument('color_qos', default_value='default'), #彩色图像的QoS。
        DeclareLaunchArgument('color_camera_info_qos', default_value='default'), #彩色图像信息的QoS。
        DeclareLaunchArgument('enable_color_auto_exposure', default_value='true'),
        DeclareLaunchArgument('enable_color_auto_exposure_priority', default_value='false'),
        DeclareLaunchArgument('color_exposure', default_value='-1'),
        DeclareLaunchArgument('color_gain', default_value='-1'),
        DeclareLaunchArgument('enable_color_auto_white_balance', default_value='true'),
        DeclareLaunchArgument('color_white_balance', default_value='-1'),
        DeclareLaunchArgument('depth_width', default_value='640'), #深度图像的宽度（像素）。
        DeclareLaunchArgument('depth_height', default_value='480'), #深度图像的高度（像素）。
        DeclareLaunchArgument('depth_fps', default_value='10'), #深度图像的帧率（fps）。
        DeclareLaunchArgument('depth_format', default_value='Y11'), #深度图像的格式，可选值有：'Y10', 'Y11', 'Y12'。
        DeclareLaunchArgument('enable_depth', default_value='true'), #是否启用深度图像。
        DeclareLaunchArgument('flip_depth', default_value='false'), #是否翻转深度图像。
        DeclareLaunchArgument('min_depth_limit', default_value='0'), #深度图像的最小深度限制。
        DeclareLaunchArgument('max_depth_limit', default_value='0'), #深度图像的最大深度限制。
        DeclareLaunchArgument('depth_qos', default_value='default'), #深度图像的QoS。
        DeclareLaunchArgument('depth_camera_info_qos', default_value='default'), #深度图像信息的QoS。
        DeclareLaunchArgument('ir_width', default_value='640'), #红外图像的宽度（像素）。
        DeclareLaunchArgument('ir_height', default_value='480'), #红外图像的高度（像素）。
        DeclareLaunchArgument('ir_fps', default_value='10'), #红外图像的帧率（fps）。
        DeclareLaunchArgument('ir_format', default_value='Y10'), #红外图像的格式，可选值有：'Y10', 'Y12'。
        DeclareLaunchArgument('enable_ir', default_value='false'), #是否启用红外图像。
        DeclareLaunchArgument('flip_ir', default_value='false'), #是否翻转红外图像。
        DeclareLaunchArgument('ir_qos', default_value='default'), #红外图像的QoS。
        DeclareLaunchArgument('ir_camera_info_qos', default_value='default'), #红外图像信息的QoS。
        DeclareLaunchArgument('enable_ir_auto_exposure', default_value='true'), 
        DeclareLaunchArgument('ir_exposure', default_value='-1'),
        DeclareLaunchArgument('ir_gain', default_value='-1'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('tf_publish_rate', default_value='0.0'),
        DeclareLaunchArgument('ir_info_url', default_value=''),
        DeclareLaunchArgument('color_info_url', default_value=''),
        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('enable_publish_extrinsic', default_value='false'),
        DeclareLaunchArgument('enable_d2c_viewer', default_value='false'),
        DeclareLaunchArgument('enable_ldp', default_value='true'),
        DeclareLaunchArgument('enable_soft_filter', default_value='true'),
        DeclareLaunchArgument('soft_filter_max_diff', default_value='-1'),
        DeclareLaunchArgument('soft_filter_speckle_size', default_value='-1'),
        DeclareLaunchArgument('ordered_pc', default_value='true'),   #过滤无效点云
        DeclareLaunchArgument('enable_depth_scale', default_value='true'),
        # DeclareLaunchArgument('align_mode', default_value='HW'),      #启用深度帧与彩色帧的对齐,硬件对其
        DeclareLaunchArgument('laser_energy_level', default_value='-1'),
        DeclareLaunchArgument('enable_heartbeat', default_value='false'),   #是否启用心跳包
    ]

    # Node configuration
    parameters = [{arg.name: LaunchConfiguration(arg.name)} for arg in args]
    compose_node = ComposableNode(
        package="orbbec_camera",
        plugin="orbbec_camera::OBCameraNodeDriver",
        name=LaunchConfiguration("camera_name"),
        namespace="",
        parameters=parameters,
    )
    # Define the ComposableNodeContainer
    container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            compose_node,
        ],
        output="screen",
    )
    # Launch description
    ld = LaunchDescription(
        args
        + [
            GroupAction(
                [PushRosNamespace(LaunchConfiguration("camera_name")), container]
            )
        ]
    )
    return ld
