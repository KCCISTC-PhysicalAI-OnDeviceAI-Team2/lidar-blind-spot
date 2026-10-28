"""/detections의 상자를 바닥 좌표로 바꿔 /camera_obstacles로 발행하는 노드."""

import math

from blindspot_calib import floor_logic
from blindspot_msgs.msg import Obstacle, ObstacleArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from vision_msgs.msg import Detection2DArray

TARGET_FRAME = 'base_jig'


class CameraToFloorNode(Node):
    """/detections 한 번마다 장애물 목록을 한 번 발행한다(없으면 빈 배열)."""

    def __init__(self, **kwargs):
        super().__init__('camera_to_floor_node', **kwargs)

        # 행렬은 config/homography.yaml(compute_homography가 만듦),
        # 발행 범위는 config/camera_to_floor_node.yaml에서 받는다(코드에 숫자를 두지 않음)
        self.declare_parameter('homography', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('image_size', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('calibrated', Parameter.Type.BOOL)
        self.declare_parameter('validation_performed', Parameter.Type.BOOL)
        self.declare_parameter('provisional', True)
        self.declare_parameter('check_points', 0)
        self.declare_parameter('check_rms_m', -1.0)
        self.declare_parameter('check_max_m', -1.0)
        self.declare_parameter('loo_rms_m', -1.0)
        self.declare_parameter('loo_max_m', -1.0)
        for name in ['x_min', 'x_max', 'y_max']:
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        self.declare_parameter('bottom_border_margin_px', Parameter.Type.DOUBLE)

        # 값이 잘못되면 여기서 멈춘다(잘못된 설정으로 조용히 돌지 않게)
        self.h = floor_logic.check_homography(self.get_parameter('homography').value)
        self.image_size = list(self.get_parameter('image_size').value)
        if len(self.image_size) != 2 or min(self.image_size) <= 0:
            raise ValueError(f'image_size는 [가로, 세로]여야 함: {self.image_size}')
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_max = self.get_parameter('y_max').value
        floor_logic.check_range(self.x_min, self.x_max, self.y_max)
        self.bottom_border_margin_px = self.get_parameter('bottom_border_margin_px').value
        if (self.bottom_border_margin_px is None
                or not math.isfinite(self.bottom_border_margin_px)
                or not 0 <= self.bottom_border_margin_px < self.image_size[1]):
            raise ValueError('bottom_border_margin_px는 0 이상 영상 높이 미만이어야 함')

        if self.get_parameter('provisional').value:
            self.get_logger().warn('임시 행렬(provisional: true). 현장 위치 정확도는 미확정')
        if not self.get_parameter('calibrated').value:
            self.get_logger().warn('homography가 격자로 구한 값이 아님(calibrated: false). '
                                   '위치가 수십 cm 틀릴 수 있음')
        elif not self.get_parameter('validation_performed').value:
            self.get_logger().warn('homography를 check 점으로 검증하지 않음'
                                   '(validation_performed: false). 임시 값으로 사용')
            loo_rms = self.get_parameter('loo_rms_m').value
            if loo_rms >= 0:
                self.get_logger().info(
                    f'homography 추정 오차(fit 점 하나씩 빼기): RMS {loo_rms:.3f} m, '
                    f'최대 {self.get_parameter("loo_max_m").value:.3f} m')
        else:
            self.get_logger().info(
                'homography check 오차: RMS '
                f'{self.get_parameter("check_rms_m").value:.3f} m, '
                f'최대 {self.get_parameter("check_max_m").value:.3f} m')

        self.create_subscription(Detection2DArray, '/detections', self.on_detections, 10)
        self.pub = self.create_publisher(ObstacleArray, '/camera_obstacles', 10)

    def on_detections(self, detections):
        msg = ObstacleArray()
        msg.header.stamp = detections.header.stamp
        msg.header.frame_id = TARGET_FRAME
        skipped = 0
        for det in detections.detections:
            if not det.results:
                skipped += 1
                continue
            box = det.bbox
            hypothesis = det.results[0].hypothesis
            if not floor_logic.valid_detection(
                    box.center.position.x, box.center.position.y,
                    box.size_x, box.size_y, hypothesis.score):
                skipped += 1
                self.get_logger().warn('크기·좌표·신뢰도가 잘못된 검출 제외', throttle_duration_sec=5.0)
                continue
            u, v = floor_logic.bottom_center(
                box.center.position.x, box.center.position.y, box.size_x, box.size_y)
            if floor_logic.outside_image(u, v, self.image_size):
                skipped += 1
                # 상자가 영상 밖이면 검출 영상 크기가 호모그래피를 구한 크기와 다를 수 있다
                self.get_logger().warn(
                    f'상자 아래 점 ({u:.0f}, {v:.0f})이 영상 크기 {self.image_size} 밖임. '
                    '검출 제외. 원본 해상도와 상자 좌표를 확인', throttle_duration_sec=5.0)
                continue
            if v >= self.image_size[1] - self.bottom_border_margin_px:
                self.get_logger().warn(
                    '화면 하단 경계 검출 유지: 바닥 접점이 가려졌다면 좌표는 실제 거리와 다름. '
                    '현재 경계 투영값이며, 가까운 물체의 정확한 거리나 검출률 검증에 사용하지 말 것',
                    throttle_duration_sec=5.0)
            found = floor_logic.box_to_floor(
                self.h, box.center.position.x, box.center.position.y, box.size_x, box.size_y,
                self.x_min, self.x_max, self.y_max, image_size=self.image_size)
            if found is None:
                continue
            obstacle = Obstacle()
            obstacle.label = hypothesis.class_id
            obstacle.x, obstacle.y = found
            obstacle.confidence = float(hypothesis.score)
            obstacle.source = 'camera'
            msg.obstacles.append(obstacle)
        self.pub.publish(msg)
        if skipped:
            self.get_logger().warn(f'결과·좌표·크기·신뢰도가 잘못된 검출 {skipped}개 제외',
                                   throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraToFloorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C나 launch 종료는 정상 종료로 본다
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
