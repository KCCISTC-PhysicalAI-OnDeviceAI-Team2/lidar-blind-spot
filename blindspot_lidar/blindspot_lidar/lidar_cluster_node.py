"""/scan의 점을 묶어 /lidar_obstacles로 발행하는 노드."""

from blindspot_lidar import cluster_logic
from blindspot_msgs.msg import Obstacle, ObstacleArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

TARGET_FRAME = 'base_jig'


class LidarClusterNode(Node):
    """/scan 한 번마다 장애물 목록을 한 번 발행한다(없으면 빈 배열)."""

    def __init__(self):
        super().__init__('lidar_cluster_node')

        # 값은 config/lidar_cluster_node.yaml에서만 받는다(코드에 숫자를 두지 않음)
        for name in ['cluster_distance', 'x_min', 'x_max', 'y_max']:
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        self.declare_parameter('min_points', Parameter.Type.INTEGER)
        self.cluster_distance = self.get_parameter('cluster_distance').value
        self.min_points = self.get_parameter('min_points').value
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_max = self.get_parameter('y_max').value
        # 값이 잘못되면 여기서 멈춘다(잘못된 설정으로 조용히 돌지 않게)
        cluster_logic.check_params(
            self.cluster_distance, self.min_points, self.x_min, self.x_max, self.y_max)

        # laser -> base_jig 변환은 /tf_static에서 받는다. 고정 값이라 한 번 받으면 계속 쓴다.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transform = None

        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(ObstacleArray, '/lidar_obstacles', 10)

    def lookup_transform(self, source_frame):
        """base_jig 기준 laser의 위치와 회전 행렬을 돌려준다. 아직 없으면 None."""
        if self.transform is not None:
            return self.transform
        if not source_frame:
            self.get_logger().warn(
                '/scan의 frame_id가 비어 있음', throttle_duration_sec=5.0)
            return None
        try:
            tf = self.tf_buffer.lookup_transform(TARGET_FRAME, source_frame, Time())
        except TransformException as error:
            self.get_logger().warn(
                f'{TARGET_FRAME} <- {source_frame} 변환을 아직 받지 못함: {error}',
                throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        try:
            rotation = cluster_logic.quaternion_to_matrix(q.x, q.y, q.z, q.w)
        except ValueError as error:
            self.get_logger().warn(
                f'{TARGET_FRAME} <- {source_frame} 변환 값이 이상함: {error}',
                throttle_duration_sec=5.0)
            return None
        self.transform = ((t.x, t.y, t.z), rotation)
        self.get_logger().info(f'{TARGET_FRAME} <- {source_frame} 변환을 받음')
        return self.transform

    def on_scan(self, scan):
        # 변환이 없으면 바닥 좌표를 알 수 없으므로 발행하지 않는다
        # (빈 배열을 내면 "장애물 없음"으로 잘못 읽힌다)
        transform = self.lookup_transform(scan.header.frame_id)
        if transform is None:
            return
        translation, rotation = transform

        # 각도 · 거리 설정이 이상한 스캔도 같은 이유로 발행하지 않는다
        try:
            points, index = cluster_logic.scan_to_points(
                scan.ranges, scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max)
        except ValueError as error:
            self.get_logger().warn(f'스캔을 건너뜀: {error}', throttle_duration_sec=5.0)
            return
        points_floor = cluster_logic.to_floor(points, translation, rotation)
        full_circle = cluster_logic.is_full_circle(len(scan.ranges), scan.angle_increment)
        found = cluster_logic.find_obstacles(
            points_floor, index, len(scan.ranges), full_circle,
            self.cluster_distance, self.min_points, self.x_min, self.x_max, self.y_max)

        msg = ObstacleArray()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = TARGET_FRAME
        for x, y in found:
            obstacle = Obstacle()
            obstacle.label = 'unknown'
            obstacle.x = x
            obstacle.y = y
            obstacle.confidence = 1.0
            obstacle.source = 'lidar'
            msg.obstacles.append(obstacle)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C나 launch 종료는 정상 종료로 본다
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
