from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNode
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_prefix = FindPackageShare('orbbec_camera_node')
    default_params_path = PathJoinSubstitution([pkg_prefix, 'config', 'camera_params.yaml'])
    
    # 声明基础参数
    declared_arguments = [
        DeclareLaunchArgument('params_file',
            default_value=default_params_path,
            description='Path to the ROS2 parameters file to use'),
        DeclareLaunchArgument('camera_name',
            default_value='camera',
            description='Name of the camera'),
        DeclareLaunchArgument('use_intra_process',
            default_value='true',
            description='Enable intra-process communication'),
    ]
    
    # 创建相机节点配置
    def get_camera_node(context):
        params_path = LaunchConfiguration('params_file').perform(context)
        return ComposableNode(
            package='orbbec_camera',
            plugin='orbbec_camera::OBCameraNodeDriver',
            name=LaunchConfiguration('camera_name'),
            parameters=[params_path],
            extra_arguments=[{'use_intra_process_comms': LaunchConfiguration('use_intra_process')}]
        )
        
    # 创建自定义处理节点配置
    def get_processor_node(context):
        params_path = LaunchConfiguration('params_file').perform(context)
        return ComposableNode(
            package='orbbec_camera_node',
            plugin='OrbbecCameraNode',
            name='orbbec_camera_node',
            parameters=[params_path],
            extra_arguments=[{'use_intra_process_comms': LaunchConfiguration('use_intra_process')}]
        )
    
    return LaunchDescription(declared_arguments)