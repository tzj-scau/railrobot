from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_prefix = FindPackageShare('orbbec_camera_node')
    
    # 包含基础启动文件
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_prefix, 'launch', 'includes', 'camera_base.launch.py')
        ])
    )
    
    # 创建组件容器
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[],
        output='screen',
    )
    
    # 加载组件节点
    load_nodes = LoadComposableNodes(
        target_container=container,
        composable_node_descriptions=[
            # Orbbec相机驱动节点
            ComposableNode(
                package='orbbec_camera',
                plugin='orbbec_camera::OBCameraNodeDriver',
                name=LaunchConfiguration('camera_name'),
                parameters=[LaunchConfiguration('params_file')],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # 自定义处理节点
            ComposableNode(
                package='orbbec_camera_node',
                plugin='OrbbecCameraNode',
                name='orbbec_camera_node',
                parameters=[LaunchConfiguration('params_file')],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
    )
    
    return LaunchDescription([
        base_launch,
        container,
        load_nodes
    ])
    
'''# 使用默认配置启动组件式节点
ros2 launch orbbec_camera_node camera_component.launch.py

# 使用自定义配置启动
ros2 launch orbbec_camera_node camera_component.launch.py params_file:=src/orbbec_camera_node/launch/params/camera_params.yaml

# 使用标准节点启动
ros2 launch orbbec_camera_node camera.launch.py
'''