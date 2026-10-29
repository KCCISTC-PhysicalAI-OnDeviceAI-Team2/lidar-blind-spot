"""
처리 노드(클러스터링, 추론, 좌표 변환, 융합)를 한 번에 켠다.

센서 드라이버와 고정 좌표 변환은 켜지 않는다(실시간에서는 sensors.launch.py가, 재생할 때는 rosbag이 내보낸다).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def config(package, name):
    """패키지의 share/config 아래 설정 파일 경로를 만든다."""
    return PathJoinSubstitution([FindPackageShare(package), 'config', name])


def generate_launch_description():
    """처리 노드를 켜는 launch를 만든다. 추론과 시각화는 인자로 켜고 끈다."""
    detector = LaunchConfiguration('detector')
    overlay = LaunchConfiguration('overlay')

    return LaunchDescription([
        DeclareLaunchArgument(
            'detector', default_value='true',
            description='추론 노드를 켠다. /detections가 든 rosbag을 재생할 때는 false로 끈다'),
        DeclareLaunchArgument(
            'overlay', default_value='false',
            description='검출 상자를 영상에 그리는 시각화 노드를 켠다'),
        Node(
            package='blindspot_lidar', executable='lidar_cluster_node',
            name='lidar_cluster_node', output='screen',
            parameters=[config('blindspot_lidar', 'lidar_cluster_node.yaml')]),
        Node(
            package='blindspot_perception', executable='detector_node',
            name='detector_node', output='screen',
            parameters=[config('blindspot_perception', 'detector_node.yaml')],
            condition=IfCondition(detector)),
        Node(
            package='blindspot_calib', executable='camera_to_floor_node',
            name='camera_to_floor_node', output='screen',
            parameters=[config('blindspot_calib', 'homography.yaml'),
                        config('blindspot_calib', 'camera_to_floor_node.yaml')]),
        Node(
            package='blindspot_fusion', executable='fusion_node',
            name='fusion_node', output='screen',
            parameters=[config('blindspot_fusion', 'fusion_node.yaml')]),
        Node(
            package='blindspot_viz', executable='overlay_node',
            name='overlay_node', output='screen',
            parameters=[config('blindspot_viz', 'overlay_node.yaml')],
            condition=IfCondition(overlay)),
    ])
