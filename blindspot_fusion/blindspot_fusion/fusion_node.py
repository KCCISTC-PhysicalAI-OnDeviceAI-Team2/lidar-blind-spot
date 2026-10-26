"""LiDAR와 카메라 장애물을 융합해 /safety_advice를 발행하는 노드."""

from blindspot_fusion import fusion_logic
from blindspot_msgs.msg import ObstacleArray, SafetyAdvice
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


def msg_time(msg):
    """메시지의 header.stamp를 초로 바꾼다. 메시지가 없으면 None."""
    if msg is None:
        return None
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class FusionNode(Node):
    """두 장애물 토픽 중 하나가 들어올 때마다 판단해서 발행한다."""

    def __init__(self):
        super().__init__('fusion_node')

        # 값은 config/fusion_node.yaml에서만 받는다(코드에 숫자를 두지 않음)
        for name in ['stop_x', 'slow_x', 'half_width', 'stale_sec', 'min_confidence']:
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        self.stop_x = self.get_parameter('stop_x').value
        self.slow_x = self.get_parameter('slow_x').value
        self.half_width = self.get_parameter('half_width').value
        self.stale_sec = self.get_parameter('stale_sec').value
        self.min_confidence = self.get_parameter('min_confidence').value

        # 소스별 가장 최근 메시지 (받은 적 없으면 None)
        self.lidar_msg = None
        self.camera_msg = None

        self.create_subscription(ObstacleArray, '/lidar_obstacles', self.on_lidar, 10)
        self.create_subscription(ObstacleArray, '/camera_obstacles', self.on_camera, 10)
        self.pub = self.create_publisher(SafetyAdvice, '/safety_advice', 10)

    def on_lidar(self, msg):
        self.lidar_msg = msg
        self.publish_advice()

    def on_camera(self, msg):
        self.camera_msg = msg
        self.publish_advice()

    def publish_advice(self):
        use_lidar, use_camera, notes = fusion_logic.select_sources(
            msg_time(self.lidar_msg), msg_time(self.camera_msg), self.stale_sec)

        advice = SafetyAdvice()
        advice.header.frame_id = 'base_jig'
        obstacles = []
        used_msgs = []
        if use_lidar:
            obstacles.extend(self.lidar_msg.obstacles)
            advice.lidar_stamp = self.lidar_msg.header.stamp
            used_msgs.append(self.lidar_msg)
        if use_camera:
            obstacles.extend(self.camera_msg.obstacles)
            advice.camera_stamp = self.camera_msg.header.stamp
            used_msgs.append(self.camera_msg)

        # header.stamp = 판단에 쓴 입력 중 가장 최근 시각
        advice.header.stamp = max(used_msgs, key=msg_time).header.stamp

        level, lidar_hit, camera_hit, worst = fusion_logic.decide(
            obstacles, self.stop_x, self.slow_x, self.half_width, self.min_confidence)
        advice.level = level
        advice.lidar_hit = lidar_hit
        advice.camera_hit = camera_hit
        advice.reason = fusion_logic.make_reason(level, worst, notes)
        self.pub.publish(advice)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
