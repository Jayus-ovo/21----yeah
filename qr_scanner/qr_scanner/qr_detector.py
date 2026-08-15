# -- Team 2 二维码检测 — ZBar 解码 + 方向编码(顺3/逆4) --
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from ai_msgs.msg import PerceptionTargets

import cv2
import numpy as np
import threading
from pyzbar.pyzbar import decode, ZBarSymbol


class QrDetector(Node):
    def __init__(self):
        super().__init__('qr_detector_node')

        qos_cfg = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cam_sub = self.create_subscription(
            CompressedImage,
            '/jpeg_img',
            self.on_camera_frame,
            qos_cfg
        )

        self.detection_sub = self.create_subscription(
            PerceptionTargets,
            '/perception_yolo_detection',
            self.on_detection_result,
            qos_cfg
        )

        self.qr_pub = self.create_publisher(
            Int32,
            '/barcode_id',
            10
        )

        self.frame_buffer = None
        self.data_lock = threading.Lock()

        self.frame_w = 640
        self.frame_h = 480

    def on_camera_frame(self, msg: CompressedImage):
        with self.data_lock:
            self.frame_buffer = msg.data

    def on_detection_result(self, msg: PerceptionTargets):
        target_rect = None
        largest_area = 0

        for obj in msg.targets:
            if obj.type != 'qrcode':
                continue
            for roi in obj.rois:
                if roi.confidence <= 0.8:
                    continue
                bottom_edge = roi.rect.y_offset + roi.rect.height
                if bottom_edge < 120 or bottom_edge > (self.frame_h - 1):
                    continue
                area = roi.rect.width * roi.rect.height
                if area > largest_area:
                    largest_area = area
                    target_rect = roi.rect

        if target_rect is None:
            return

        jpeg_bytes = None
        with self.data_lock:
            if self.frame_buffer is None:
                return
            jpeg_bytes = self.frame_buffer

        raw_arr = np.frombuffer(jpeg_bytes, np.uint8)
        gray = cv2.imdecode(raw_arr, cv2.IMREAD_GRAYSCALE)

        if gray is None:
            return

        cx = target_rect.x_offset + target_rect.width / 2.0
        cy = target_rect.y_offset + target_rect.height / 2.0
        pad_w = target_rect.width * 1.2
        pad_h = target_rect.height * 1.2

        x0 = max(0, int(cx - pad_w / 2.0))
        y0 = max(0, int(cy - pad_h / 2.0))
        x1 = min(self.frame_w, int(cx + pad_w / 2.0))
        y1 = min(self.frame_h, int(cy + pad_h / 2.0))

        if x1 <= x0 or y1 <= y0:
            return

        roi_crop = gray[y0:y1, x0:x1]

        results = decode(roi_crop, symbols=[ZBarSymbol.QRCODE])

        for code in results:
            payload = code.data.decode("utf-8")
            self.get_logger().info(f"QR payload: {payload}")

            out_val = None

            if payload == "ClockWise":
                out_val = 3
            elif payload == "AntiClockWise":
                out_val = 4
            else:
                try:
                    val = int(payload)
                    if 1 <= val <= 9999:
                        out_val = 3 if val % 2 != 0 else 4
                except ValueError:
                    pass

            if out_val is not None:
                out_msg = Int32()
                out_msg.data = out_val
                self.qr_pub.publish(out_msg)
                break


def main(args=None):
    rclpy.init(args=args)
    node = QrDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()