import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def load_yaml(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    package_name = 'rail_robot_inspection'
    
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'config.yaml'
    )

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the YAML configuration file'
    )

    def create_launch_nodes(context, *args, **kwargs):
        config_file_path_resolved = LaunchConfiguration('config_file').perform(context)
        
        if not os.path.isfile(config_file_path_resolved):
            raise FileNotFoundError(f"Configuration file '{config_file_path_resolved}' not found.")

        yaml_params = load_yaml(config_file_path_resolved) if config_file_path_resolved else {}

        # 创建节点列表
        nodes = []

        # 1. 首先添加串口桥接节点
        if 'serial_bridge' in yaml_params:
            nodes.append(Node(
                package=package_name,
                executable='serial_bridge_node',
                name='serial_bridge_node',
                output='screen',
                parameters=[yaml_params['serial_bridge']]
            ))

        # 2. 延迟启动相机容器 (等待5秒让串口桥接节点完全启动)
        orbbec_camera_params = yaml_params.get('orbbec_camera', {})
        camera_container = ComposableNodeContainer(
            name='camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='orbbec_camera',
                    plugin='orbbec_camera::OBCameraNodeDriver',
                    name='ob_camera_node',
                    parameters=[orbbec_camera_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='orbbec_camera_node',
                    plugin='orbbec_camera_node::OrbbecCameraNode',  # 注意这里使用双冒号
                    name='orbbec_camera_node',
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output='screen',
        )
        nodes.append(TimerAction(
            period=5.0,  # 5秒延迟
            actions=[camera_container]
        ))

        # 3. 延迟启动导轨控制节点 (等待7秒让串口和相机节点完全启动)
        if 'rail_robot' in yaml_params:
            rail_robot_params = yaml_params['rail_robot']
            rail_control_node = Node(
                package=package_name,
                executable='rail_control_node',
                name='rail_control_node',
                output='screen',
                parameters=[{
                    'positions': rail_robot_params.get('positions', []),
                    'inspection_mode': rail_robot_params.get('inspection_mode', 'immediate'),
                    'immediate_interval': rail_robot_params.get('immediate_interval', 10.0),
                    'daily_schedule': rail_robot_params.get('daily_schedule', [])
                }]
            )
            nodes.append(TimerAction(
                period=7.0,  # 7秒延迟
                actions=[rail_control_node]
            ))

        # 4. 延迟启动热成像相机节点 (等待6秒让串口和相机节点完全启动)
        if 'thermal_camera' in yaml_params:
            thermal_camera_node = Node(
                package=package_name,
                executable='thermal_camera_node',
                name='thermal_camera_node',
                output='screen',
                parameters=[yaml_params['thermal_camera']]
            )
            nodes.append(TimerAction(
                period=6.0,  # 6秒延迟
                actions=[thermal_camera_node]
            ))

        return nodes

    return LaunchDescription([
        declare_config_file,
        OpaqueFunction(function=create_launch_nodes)
    ])