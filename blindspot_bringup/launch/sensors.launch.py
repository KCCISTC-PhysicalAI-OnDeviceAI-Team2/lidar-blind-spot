"""센서 지그의 LiDAR, 카메라, 고정 좌표 변환을 켠다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# base_jig 기준 센서 위치 [m]와 각도 [rad].
# 지그에서 잰 값으로 바꾼다. 지금은 임시 값 0.
LASER_TF = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
CAMERA_TF = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}


def static_tf(child, tf):
    """base_jig -> child 고정 변환을 내보내는 노드를 만든다."""
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_jig_to_' + child,
        arguments=[
            '--x', str(tf['x']),
            '--y', str(tf['y']),
            '--z', str(tf['z']),
            '--yaw', str(tf['yaw']),
            '--pitch', str(tf['pitch']),
            '--roll', str(tf['roll']),
            '--frame-id', 'base_jig',
            '--child-frame-id', child,
        ],
    )


def generate_launch_description():
    """LiDAR, 카메라, 고정 좌표 변환을 켜는 launch를 만든다."""
    use_lidar = LaunchConfiguration('use_lidar')
    use_camera = LaunchConfiguration('use_camera')

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py',
        ])),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser',
        }.items(),
        condition=IfCondition(use_lidar),
    )

    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='camera',
        parameters=[PathJoinSubstitution([
            FindPackageShare('blindspot_bringup'), 'config', 'camera.yaml',
        ])],
        condition=IfCondition(use_camera),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='Start the LiDAR driver'),
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='Start the camera node'),
        lidar,
        camera,
        static_tf('laser', LASER_TF),
        static_tf('camera', CAMERA_TF),
    ])
