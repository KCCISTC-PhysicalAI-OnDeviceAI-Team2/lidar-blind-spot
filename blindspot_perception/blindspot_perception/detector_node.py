"""Run YOLO on camera images and publish 2D detections."""

import os

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class DetectorNode(Node):
    """Subscribe to images, run YOLO, publish Detection2DArray."""

    def __init__(self):
        super().__init__('detector_node')

        # Parameters (values come from config/detector_node.yaml)
        self.declare_parameter('model_path', '~/models/yolo11n.pt')
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('imgsz', 640)

        model_path = os.path.expanduser(
            self.get_parameter('model_path').value)
        self.conf = float(self.get_parameter('conf_threshold').value)
        self.imgsz = int(self.get_parameter('imgsz').value)

        if not os.path.isfile(model_path):
            raise FileNotFoundError(f'model not found: {model_path}')

        self.get_logger().info(f'loading model: {model_path}')
        self.model = YOLO(model_path)

        # Warm-up: the first inference is slow (GPU init), do it before frames arrive
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        self.model.predict(dummy, imgsz=self.imgsz, verbose=False)
        self.get_logger().info('model ready')

        self.bridge = CvBridge()

        # Sensor QoS (best effort) with depth 1: keep only the newest image
        sub_qos = QoSProfile(
            reliability=qos_profile_sensor_data.reliability,
            history=qos_profile_sensor_data.history,
            depth=1,
        )
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, sub_qos)

        # Result topic: default QoS (reliable, depth 10)
        self.pub = self.create_publisher(Detection2DArray, '/detections', 10)

    def image_callback(self, msg):
        """Run detection on one image and publish the result."""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        result = self.model.predict(
            frame, conf=self.conf, imgsz=self.imgsz, verbose=False)[0]

        out = Detection2DArray()
        out.header = msg.header  # same stamp and frame as the input image

        names = result.names
        for xywh, cls, score in zip(result.boxes.xywh.tolist(),
                                    result.boxes.cls.tolist(),
                                    result.boxes.conf.tolist()):
            det = Detection2D()
            det.header = msg.header

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = names[int(cls)]
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)

            # xywh = box center x, y and width, height in original image pixels
            det.bbox.center.position.x = float(xywh[0])
            det.bbox.center.position.y = float(xywh[1])
            det.bbox.size_x = float(xywh[2])
            det.bbox.size_y = float(xywh[3])

            out.detections.append(det)

        self.pub.publish(out)  # empty array when nothing is found


def main(args=None):
    """Start the node."""
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
