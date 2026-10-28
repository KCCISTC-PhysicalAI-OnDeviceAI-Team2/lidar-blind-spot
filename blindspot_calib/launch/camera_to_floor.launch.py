"""camera_to_floor_node만 설정 파일 두 개와 함께 켠다(카메라와 추론 노드는 켜지 않음)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """config/homography.yaml과 config/camera_to_floor_node.yaml을 불러와 노드를 켠다."""
    config = PathJoinSubstitution([FindPackageShare('blindspot_calib'), 'config'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution([config, 'homography.yaml']),
            description='호모그래피 파라미터 YAML 경로'),
        DeclareLaunchArgument(
            'range_params_file',
            default_value=PathJoinSubstitution([config, 'camera_to_floor_node.yaml']),
            description='발행 범위와 경계 경고 설정 YAML 경로'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='blindspot_calib',
            executable='camera_to_floor_node',
            name='camera_to_floor_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                LaunchConfiguration('range_params_file'),
                {'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
        ),
    ])
