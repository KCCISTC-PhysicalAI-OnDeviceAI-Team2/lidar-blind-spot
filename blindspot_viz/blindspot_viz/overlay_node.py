import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

PALETTE = [(0, 255, 0), (0, 165, 255), (255, 0, 255), (255, 255, 0), (0, 0, 255)]


class OverlayNode(Node):
    """Draw the latest detections on each camera image (display only)."""

    def __init__(self):
        super().__init__('overlay_node')
        self.declare_parameter('max_age_sec', 0.5)
        self.declare_parameter('line_thickness', 2)
        self.declare_parameter('font_scale', 0.6)
        self.max_age = float(self.get_parameter('max_age_sec').value)
        self.thickness = int(self.get_parameter('line_thickness').value)
        self.font_scale = float(self.get_parameter('font_scale').value)

        self.bridge = CvBridge()
        self.latest = None  # latest Detection2DArray
        self.colors = {}    # class name -> BGR color

        self.create_subscription(
            Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            Detection2DArray, '/detections', self.on_detections, 10)
        self.pub = self.create_publisher(Image, '/detections_overlay', 10)

    def on_detections(self, msg):
        self.latest = msg

    def color_for(self, name):
        if name not in self.colors:
            self.colors[name] = PALETTE[len(self.colors) % len(PALETTE)]
        return self.colors[name]

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.latest is not None:
            diff = Time.from_msg(msg.header.stamp) - Time.from_msg(self.latest.header.stamp)
            # Skip boxes that are too far from this image in time
            if abs(diff.nanoseconds) / 1e9 <= self.max_age:
                for det in self.latest.detections:
                    self.draw(frame, det)
        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = msg.header
        self.pub.publish(out)

    def draw(self, frame, det):
        cx = det.bbox.center.position.x
        cy = det.bbox.center.position.y
        w = det.bbox.size_x
        h = det.bbox.size_y
        x1, y1 = int(cx - w / 2), int(cy - h / 2)
        x2, y2 = int(cx + w / 2), int(cy + h / 2)

        name, score = '?', 0.0
        if det.results:
            name = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score
        color = self.color_for(name)

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, self.thickness)
        cv2.putText(frame, f'{name} {score:.2f}', (x1, max(y1 - 5, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, color, self.thickness)


def main(args=None):
    rclpy.init(args=args)
    node = OverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
