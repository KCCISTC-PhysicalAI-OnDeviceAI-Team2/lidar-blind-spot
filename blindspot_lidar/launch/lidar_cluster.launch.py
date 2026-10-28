"""lidar_cluster_node만 설정 파일과 함께 켠다(LiDAR 드라이버와 고정 변환은 켜지 않음)."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """config/lidar_cluster_node.yaml을 불러와 노드를 켜는 launch를 만든다."""
    return LaunchDescription([
        Node(
            package='blindspot_lidar',
            executable='lidar_cluster_node',
            name='lidar_cluster_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('blindspot_lidar'), 'config', 'lidar_cluster_node.yaml',
            ])],
        ),
    ])
